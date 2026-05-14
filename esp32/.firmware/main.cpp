/*
 * main.cpp — Tesla FSD Unlock for ESP32
 *
 * Port of hypery11/flipper-tesla-fsd to M5Stack ATOM Lite + ATOMIC CAN Base.
 *
 * Default state: Listen-Only (blue LED).  Press button once to go Active (green).
 *
 * Button:
 *   Single click  → toggle Listen-Only / Active
 *   Long press 3s → toggle NAG Killer on/off
 *   Double click  → toggle BMS serial output
 *
 * Serial 115200 baud.  Status prints every 5 s when Active.
 * BMS output (when enabled): voltage, current, power, SoC, temp every 1 s.
 */

#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include "config.h"
#include "fsd_handler.h"
#include "can_driver.h"
#include "led.h"
#include "wifi_manager.h"
#include "web_dashboard.h"
#include "can_dump.h"
#include "prefs.h"

// ── Globals ───────────────────────────────────────────────────────────────────
static TaskHandle_t g_can_task_handle = nullptr;
static CanDriver *g_can   = nullptr;
static FSDState   g_state = {};
static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

static void state_enter() {
    portENTER_CRITICAL(&g_state_mux);
}

static void state_exit() {
    portEXIT_CRITICAL(&g_state_mux);
}

static FSDState state_snapshot() {
    FSDState s;
    state_enter();
    s = g_state;
    state_exit();
    return s;
}

static void apply_detected_hw(TeslaHWVersion hw, const char *reason) {
    if (hw == TeslaHW_Unknown) return;

    state_enter();
    if (!g_state.hw_mode_auto) {
        state_exit();
        return;
    }
    if (g_state.hw_version == hw) {
        state_exit();
        return;
    }
    fsd_apply_hw_version(&g_state, hw);
    state_exit();

    const char *hw_str =
        (hw == TeslaHW_HW4) ? "HW4" :
        (hw == TeslaHW_HW3) ? "HW3" : "Legacy";
    Serial.printf("[HW] Auto-detected: %s (%s)\n", hw_str, reason);
    can_dump_log("HW  auto-detected: %s (%s)", hw_str, reason);
}

// ── Button state machine ──────────────────────────────────────────────────────
static uint32_t g_btn_down_ms     = 0;
static uint32_t g_last_release_ms = 0;
static bool     g_btn_down        = false;
static int      g_pending_clicks  = 0;
static bool     g_long_fired      = false;  // prevent double-fire on long press
static bool     g_btn_ignore_boot = true;   // wait for release after boot
static bool     g_factory_reset_window   = false;  // set true on clean boot, clears at 20s
static bool     g_factory_reset_eligible = false;  // latched at leading edge if press was in window
static bool     g_factory_reset_armed    = false;  // blink done, waiting for release

#if defined(BOARD_LILYGO)
static uint32_t g_last_can_rx_ms = 0;
static bool     g_sleep_warned   = false;
#endif

static void dispatch_clicks(int n) {
    if (n == 1) {
        // Toggle Listen-Only ↔ Active
        FSDState saved;
        bool active;
        state_enter();
        if (g_state.op_mode == OpMode_ListenOnly) {
            g_state.op_mode = OpMode_Active;
            active = true;
        } else {
            g_state.op_mode = OpMode_ListenOnly;
            active = false;
        }
        saved = g_state;
        state_exit();
        g_can->setListenOnly(!active);
        Serial.println(active ? "[BTN] → Active mode" : "[BTN] → Listen-Only mode");
        can_dump_log(active ? "MODE switched to Active — TX enabled" : "MODE switched to Listen-Only — TX disabled");
        prefs_save(&saved);
    } else if (n >= 2) {
        // Toggle BMS serial output
        FSDState saved;
        bool enabled;
        state_enter();
        g_state.bms_output = !g_state.bms_output;
        enabled = g_state.bms_output;
        saved = g_state;
        state_exit();
        Serial.printf("[BTN] BMS output: %s\n", enabled ? "ON" : "OFF");
        prefs_save(&saved);
    }
}

static void button_tick() {
    bool pressed = (digitalRead(PIN_BUTTON) == LOW);
    uint32_t now = millis();

    if (g_btn_ignore_boot) {
        if (!pressed) g_btn_ignore_boot = false;
        return;
    }

    if (pressed && !g_btn_down) {
        // Leading edge — debounce
        if ((now - g_last_release_ms) < BUTTON_DEBOUNCE_MS) return;
        g_btn_down             = true;
        g_btn_down_ms          = now;
        g_long_fired           = false;
        g_factory_reset_eligible = g_factory_reset_window;  // latch at press time
    }

    if (g_btn_down && pressed && !g_long_fired) {
        uint32_t held = now - g_btn_down_ms;
        if (g_factory_reset_eligible && held >= FACTORY_RESET_HOLD_MS) {
            g_long_fired           = true;
            g_pending_clicks       = 0;
            g_factory_reset_armed  = true;
            Serial.println("[BTN] Factory reset armed — release to confirm");
            led_factory_blink();
        } else if (!g_factory_reset_eligible && held >= LONG_PRESS_MS) {
            g_long_fired      = true;
            g_pending_clicks  = 0;
            FSDState saved;
            bool enabled;
            state_enter();
            g_state.nag_killer = !g_state.nag_killer;
            enabled = g_state.nag_killer;
            saved = g_state;
            state_exit();
            Serial.printf("[BTN] NAG Killer: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
        // eligible press with held < FACTORY_RESET_HOLD_MS: suppress 3s NAG killer
    }

    if (!pressed && g_btn_down) {
        // Trailing edge
        g_btn_down               = false;
        g_last_release_ms        = now;
        g_factory_reset_eligible = false;
        if (g_factory_reset_armed) {
            Serial.println("[BTN] Factory reset confirmed — clearing NVS");
            prefs_clear();
            delay(200);
            ESP.restart();
        }
        if (!g_long_fired) {
            g_pending_clicks++;
        }
    }

    // Flush pending clicks after the double-click window closes
    if (g_pending_clicks > 0 && !g_btn_down &&
        (now - g_last_release_ms) >= DOUBLE_CLICK_MS) {
        dispatch_clicks(g_pending_clicks);
        g_pending_clicks = 0;
    }
}

static bool send_can_frame(const CanFrame& frame) {
    if (g_can != nullptr && g_can->send(frame)) {
        state_enter();
        g_state.frames_sent++;
        state_exit();
        return true;
    } else {
        state_enter();
        g_state.tx_fail_count++;
        state_exit();
        return false;
    }
}

static bool send_can_frame_if_allowed(const CanFrame& frame) {
    FSDState state = state_snapshot();
    if (!fsd_can_transmit(&state)) {
        state_enter();
        g_state.tx_fail_count++;
        state_exit();
        return false;
    }
    return send_can_frame(frame);
}

static uint8_t decode_hw4_mux2_offset_raw(const CanFrame& frame) {
    return (uint8_t)(frame.data[1] & 0x3Fu);
}

static void log_hw4_mux2_debug(const CanFrame& in,
                               const CanFrame& out,
                               bool modified,
                               bool tx_allowed,
                               bool sent) {
    static uint32_t last_log_ms = 0;
    static uint8_t last_raw_in = 0xFFu;
    static uint8_t last_raw_out = 0xFFu;
    static uint8_t last_target = 0xFFu;
    static bool last_modified = false;
    static bool last_tx_allowed = false;
    static bool last_sent = false;

    uint8_t raw_in = decode_hw4_mux2_offset_raw(in);
    uint8_t raw_out = decode_hw4_mux2_offset_raw(out);
    FSDState state = state_snapshot();
    uint8_t target = state.hw4_offset_active;
    uint32_t now = millis();
    bool changed =
        raw_in != last_raw_in ||
        raw_out != last_raw_out ||
        target != last_target ||
        modified != last_modified ||
        tx_allowed != last_tx_allowed ||
        sent != last_sent;

    if (!changed && (now - last_log_ms) < 1000u) return;

    char debug_log[224];
    snprintf(debug_log, sizeof(debug_log),
             "HW4 0x3FD mux2 b0:%02X->%02X b1:%02X->%02X raw:%u->%u "
             "target=%u profile=%u limit=%u tx=%u mod=%u sent=%u",
             in.data[0], out.data[0], in.data[1], out.data[1],
             raw_in, raw_out,
             target,
             (unsigned)state.speed_profile,
             (unsigned)state.das_speed_limit_active * 5u,
             tx_allowed ? 1 : 0,
             modified ? 1 : 0,
             sent ? 1 : 0);
    state_enter();
    strncpy(g_state.web_debug_log, debug_log, sizeof(g_state.web_debug_log));
    g_state.web_debug_log[sizeof(g_state.web_debug_log) - 1] = '\0';
    state_exit();

    Serial.printf("[DBG] HW4 0x3FD mux2 b0:%02X->%02X b1:%02X->%02X "
                  "raw:%u->%u target=%u profile=%u limit=%u tx=%u mod=%u sent=%u\n",
                  in.data[0], out.data[0], in.data[1], out.data[1],
                  raw_in, raw_out,
                  target,
                  (unsigned)state.speed_profile,
                  (unsigned)state.das_speed_limit_active * 5u,
                  tx_allowed ? 1 : 0,
                  modified ? 1 : 0,
                  sent ? 1 : 0);
    can_dump_log("DBG HW4 0x3FD mux2 b0:%02X->%02X b1:%02X->%02X "
                 "raw:%u->%u target=%u profile=%u limit=%u tx=%u mod=%u sent=%u",
                 in.data[0], out.data[0], in.data[1], out.data[1],
                 raw_in, raw_out,
                 target,
                 (unsigned)state.speed_profile,
                 (unsigned)state.das_speed_limit_active * 5u,
                 tx_allowed ? 1 : 0,
                 modified ? 1 : 0,
                 sent ? 1 : 0);

    last_log_ms = now;
    last_raw_in = raw_in;
    last_raw_out = raw_out;
    last_target = target;
    last_modified = modified;
    last_tx_allowed = tx_allowed;
    last_sent = sent;
}

// ── LED refresh ───────────────────────────────────────────────────────────────
static void update_led() {
    FSDState state = state_snapshot();
    if (g_factory_reset_armed) {
        led_set(LED_WHITE);
        return;
    }
    if (state.rx_count == 0 && millis() > WIRING_WARN_MS) {
        led_set(LED_RED);
    } else if (state.tesla_ota_in_progress) {
        led_set(LED_YELLOW);
    } else if (state.op_mode == OpMode_Active) {
        led_set(LED_GREEN);
    } else {
        led_set(LED_BLUE);
    }
}

// ── CAN frame dispatcher ──────────────────────────────────────────────────────
static void process_frame(const CanFrame &frame) {
    bool saw_gtw_car_state = false;
    bool saw_gtw_car_config = false;
    bool saw_ap_control = false;
    bool saw_bms_hv = false;
    bool saw_bms_soc = false;
    bool saw_bms_thermal = false;

    state_enter();
    g_state.rx_count++;
    if (frame.id == CAN_ID_GTW_CAR_STATE)  { g_state.seen_gtw_car_state++;  saw_gtw_car_state = true; }
    if (frame.id == CAN_ID_GTW_CAR_CONFIG) { g_state.seen_gtw_car_config++; saw_gtw_car_config = true; }
    if (frame.id == CAN_ID_AP_CONTROL)     { g_state.seen_ap_control++;     saw_ap_control = true; }
    if (frame.id == CAN_ID_BMS_HV_BUS)     { g_state.seen_bms_hv++;         saw_bms_hv = true; }
    if (frame.id == CAN_ID_BMS_SOC)        { g_state.seen_bms_soc++;        saw_bms_soc = true; }
    if (frame.id == CAN_ID_BMS_THERMAL)    { g_state.seen_bms_thermal++;    saw_bms_thermal = true; }
    state_exit();

    can_dump_record(frame);
#if defined(BOARD_LILYGO)
    g_last_can_rx_ms = millis();
    g_sleep_warned   = false;
#endif

    // DLC sanity: skip zero-length frames
    if (frame.dlc == 0) return;

    // ── HW auto-detect (passive, runs in both modes) ─────────────────────────
    if (saw_gtw_car_config) {
        TeslaHWVersion hw = fsd_detect_hw_version(&frame);
        FSDState state = state_snapshot();
        if (hw != TeslaHW_Unknown && state.hw_version == TeslaHW_Unknown && state.hw_mode_auto)
            apply_detected_hw(hw, "0x398");
        return;
    }

    // ── OTA monitoring (always, mode-independent) ─────────────────────────────
    if (saw_gtw_car_state) {
        bool was_ota;
        bool is_ota;
        uint8_t raw;
        state_enter();
        was_ota = g_state.tesla_ota_in_progress;
        fsd_handle_gtw_car_state(&g_state, &frame);
        is_ota = g_state.tesla_ota_in_progress;
        raw = g_state.ota_raw_state;
        state_exit();
        if (!was_ota && is_ota) {
            Serial.printf("[OTA] Update in progress (raw=%u) - TX suspended\n", raw);
            can_dump_log("OTA  started — TX suspended");
        } else if (was_ota && !is_ota) {
            Serial.printf("[OTA] Update finished (raw=%u) - TX resumed\n", raw);
            can_dump_log("OTA  finished — TX resumed");
        }
        return;
    }

    // ── BMS sniff (read-only, always) ─────────────────────────────────────────
    if (saw_bms_hv)      { state_enter(); fsd_handle_bms_hv(&g_state, &frame);      state_exit(); return; }
    if (saw_bms_soc)     { state_enter(); fsd_handle_bms_soc(&g_state, &frame);     state_exit(); return; }
    if (saw_bms_thermal) { state_enter(); fsd_handle_bms_thermal(&g_state, &frame); state_exit(); return; }

    // ── DAS status (read-only, always) — gating for NAG killer ───────────────
    if (frame.id == CAN_ID_DAS_STATUS)  { state_enter(); fsd_handle_das_status(&g_state, &frame);  state_exit(); return; }
    if (frame.id == CAN_ID_ISA_SPEED)   { state_enter(); fsd_handle_isa_speed_status(&g_state, &frame); state_exit(); }

    // ── Beyond here only run when TX is allowed ───────────────────────────────
    FSDState state = state_snapshot();
    bool tx = fsd_can_transmit(&state);

    // NAG killer — build echo and send before the real frame propagates (0x370)
    if (frame.id == CAN_ID_EPAS_STATUS) {
        CanFrame echo;
        state_enter();
        bool fired = fsd_handle_nag_killer(&g_state, &frame, &echo);
        state_exit();
        if (fired) {
            uint8_t lvl     = (frame.data[4] >> 6) & 0x03;
            uint8_t cnt_in  = frame.data[6] & 0x0F;
            uint8_t cnt_out = echo.data[6] & 0x0F;
            can_dump_log("NAG 0x370 hands_off lvl=%u cnt=%u->%u %s",
                         lvl, cnt_in, cnt_out, tx ? "TX echo" : "listen-only no-TX");
            if (tx) send_can_frame_if_allowed(echo);
        }
        return;
    }

    // Legacy stalk (0x045) — updates speed_profile, no TX
    state = state_snapshot();
    if (frame.id == CAN_ID_STW_ACTN_RQ && state.hw_version == TeslaHW_Legacy) {
        state_enter();
        fsd_handle_legacy_stalk(&g_state, &frame);
        state_exit();
        return;
    }

    // Legacy autopilot control (0x3EE)
    if (frame.id == CAN_ID_AP_LEGACY && state.hw_version == TeslaHW_Legacy) {
        CanFrame f = frame;
        state_enter();
        bool modified = fsd_handle_legacy_autopilot(&g_state, &f);
        state_exit();
        if (modified && tx)
            send_can_frame_if_allowed(f);
        return;
    }

    // Auto-upgrade Legacy→HW3: Palladium S/X with HW3 reports das_hw=0
    // (→Legacy) but actually uses 0x3FD. True Legacy never broadcasts 0x3FD.
    if (state.hw_mode_auto && state.hw_version == TeslaHW_Legacy && saw_ap_control) {
        apply_detected_hw(TeslaHW_HW3, "upgrade:Legacy→HW3(0x3FD seen)");
        state = state_snapshot();
    }

    // Fallback HW detection when 0x398 is unavailable on the tapped bus.
    // Delay 0x3FD→HW3 fallback to avoid misclassifying HW4 (which also has
    // 0x3FD) before 0x399 arrives. 0x3EE and 0x399 are unambiguous.
    static uint32_t hw_fallback_3fd_count = 0;
    static TeslaHWVersion hw_fallback_last_hw = TeslaHW_Unknown;
    if (state.hw_version != hw_fallback_last_hw) {
        hw_fallback_3fd_count = 0;
        hw_fallback_last_hw = state.hw_version;
    }
    if (state.hw_mode_auto && state.hw_version == TeslaHW_Unknown) {
        if (frame.id == CAN_ID_AP_LEGACY) {
            apply_detected_hw(TeslaHW_Legacy, "fallback:0x3EE");
        } else if (frame.id == CAN_ID_ISA_SPEED) {
            apply_detected_hw(TeslaHW_HW4, "fallback:0x399");
            hw_fallback_3fd_count = 0;
        } else if (saw_ap_control) {
            hw_fallback_3fd_count++;
            if (hw_fallback_3fd_count >= 50)
                apply_detected_hw(TeslaHW_HW3, "fallback:0x3FD(confirmed)");
        }
        state = state_snapshot();
    }

    // ISA speed chime (0x399, HW4 only)
    if (frame.id == CAN_ID_ISA_SPEED &&
        state.hw_version == TeslaHW_HW4 &&
        state.suppress_speed_chime) {
        CanFrame f = frame;
        if (fsd_handle_isa_speed_chime(&f) && tx)
            send_can_frame_if_allowed(f);
        return;
    }

    // Follow distance → speed_profile (0x3F8), no TX
    if (frame.id == CAN_ID_FOLLOW_DIST) {
        state_enter();
        if (g_state.profile_mode_auto) {
            fsd_handle_follow_distance(&g_state, &frame);
        }
        state_exit();
        return;
    }

    // TLSSC Restore (0x331) — DAS config spoof
    if (frame.id == CAN_ID_DAS_AP_CONFIG) {
        CanFrame f = frame;
        state_enter();
        bool modified = fsd_handle_tlssc_restore(&g_state, &f);
        state_exit();
        if (modified && tx)
            send_can_frame_if_allowed(f);
        return;
    }

    // HW3/HW4 autopilot control (0x3FD) — main FSD activation frame
    if (saw_ap_control) {
        CanFrame f = frame;
        state = state_snapshot();
        bool is_hw4_mux2 = (state.hw_version == TeslaHW_HW4 &&
                            frame.dlc >= 8 &&
                            ((frame.data[0] & 0x07u) == 2u));
        state_enter();
        bool modified = fsd_handle_autopilot_frame(&g_state, &f);
        state_exit();
        bool sent = false;
        if (modified && tx)
            sent = send_can_frame_if_allowed(f);
        if (is_hw4_mux2)
            log_hw4_mux2_debug(frame, f, modified, tx, sent);
        return;
    }
}

#if defined(BOARD_LILYGO)
// ── Deep-sleep watchdog (Lilygo only) ────────────────────────────────────────
static void sleep_tick(uint32_t now) {
    if (now < g_last_can_rx_ms) return;
    uint32_t idle_ms = now - g_last_can_rx_ms;
    FSDState state = state_snapshot();

    if (idle_ms >= state.sleep_idle_ms) {
        Serial.printf("[SLEEP] Entering deep sleep after %lu ms CAN silence\n",
                      (unsigned long)idle_ms);
        can_dump_stop();
        sd_syslog_close();
        led_set(LED_SLEEP);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_CAN_RX, 0);
        esp_deep_sleep_start();
        // never returns
    } else if (!g_sleep_warned && idle_ms >= (state.sleep_idle_ms - SLEEP_WARN_MS)) {
        g_sleep_warned = true;
        uint32_t remaining_ms = state.sleep_idle_ms - idle_ms;
        Serial.printf("[SLEEP] Warning: %lu ms idle, sleeping in %lu ms\n",
                      (unsigned long)idle_ms, (unsigned long)remaining_ms);
    }
}
#endif

// ── CAN task (Core 1) ─────────────────────────────────────────────────────────
static void can_task(void *param) {
    (void)param;
    Serial.printf("[CAN] Task started on Core %d\n", xPortGetCoreID());

    while (true) {
        uint32_t now = millis();

        if (g_factory_reset_window && now >= FACTORY_RESET_WINDOW_MS) {
            g_factory_reset_window = false;
            Serial.println("[BTN] Factory reset window closed");
        }

        button_tick();

        // Drain all available CAN frames in one shot
        CanFrame frame;
        while (g_can->receive(frame)) {
            process_frame(frame);
        }

        // ── Periodic error counter refresh (~every 250 ms) ────────────────────
        static uint32_t last_err_ms = 0;
        if ((now - last_err_ms) >= 250u) {
            uint32_t crc_errors = g_can->errorCount();
            state_enter();
            g_state.crc_err_count = crc_errors;
            state_exit();
            last_err_ms = now;
        }

        // ── Precondition frame injection ──────────────────────────────────────
        static uint32_t last_precond_ms = 0;
        FSDState state = state_snapshot();
        if (state.precondition && fsd_can_transmit(&state) &&
            (now - last_precond_ms) >= PRECOND_INTERVAL_MS) {
            CanFrame pf;
            fsd_build_precondition_frame(&pf);
            send_can_frame_if_allowed(pf);
            last_precond_ms = now;
        }

        // ── BMS serial output ─────────────────────────────────────────────────
        static uint32_t last_bms_ms = 0;
        state = state_snapshot();
        if (state.bms_output && state.bms_seen &&
            (now - last_bms_ms) >= BMS_PRINT_MS) {
            float kw = state.pack_voltage_v * state.pack_current_a / 1000.0f;
            Serial.printf("[BMS] %.1fV  %.1fA  %.2fkW  SoC:%.1f%%  Temp:%d~%d°C\n",
                state.pack_voltage_v,
                state.pack_current_a,
                kw,
                state.soc_percent,
                (int)state.batt_temp_min_c,
                (int)state.batt_temp_max_c);
            last_bms_ms = now;
        }

        // ── Active-mode status line ───────────────────────────────────────────
        static uint32_t last_status_ms = 0;
        state = state_snapshot();
        if (state.op_mode == OpMode_Active &&
            (now - last_status_ms) >= STATUS_PRINT_MS) {
            const char *hw_str =
                (state.hw_version == TeslaHW_HW4)    ? "HW4"    :
                (state.hw_version == TeslaHW_HW3)    ? "HW3"    :
                (state.hw_version == TeslaHW_Legacy)  ? "Legacy" : "?";
            Serial.printf(
                "[STA] HW:%-6s FSD:%-4s NAG:%-10s OTA:%-3s "
                "Profile:%d  RX:%lu TX:%lu Err:%lu\n",
                hw_str,
                state.fsd_enabled     ? "ON"         : "wait",
                state.nag_suppressed  ? "suppressed"  : "active",
                state.tesla_ota_in_progress ? "YES"  : "no",
                state.speed_profile,
                (unsigned long)state.rx_count,
                (unsigned long)state.frames_modified,
                (unsigned long)state.crc_err_count);
            last_status_ms = now;
        }

        // ── Wiring sanity warning ─────────────────────────────────────────────
        static uint32_t last_warn_ms = 0;
        state = state_snapshot();
        if (state.rx_count == 0 && now > WIRING_WARN_MS &&
            (now - last_warn_ms) >= 2000u) {
            Serial.println("[WARN] No CAN traffic after 5 s — check wiring");
            Serial.println("[WARN] Verify CAN-H on OBD pin 6, CAN-L on pin 14");
            last_warn_ms = now;
        }

        can_dump_tick(now);

#if defined(BOARD_LILYGO)
        sleep_tick(now);
#endif

        update_led();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
#if defined(BOARD_LILYGO)
    g_last_can_rx_ms = millis();
#endif
    Serial.begin(115200);
    delay(300);

    Serial.println("\n============================");
    Serial.println(" Tesla FSD Unlock — ESP32   ");
    Serial.println("============================");
    Serial.printf("[FSD] Build: %s %s\n", __DATE__, __TIME__);

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        Serial.printf("[OTA] Running from: %s\n", running->label);

        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                Serial.println("[OTA] First boot after update - marking as valid...");
                if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
                    Serial.println("[OTA] Firmware marked valid");
                } else {
                    Serial.println("[OTA] WARNING: Could not mark firmware valid");
                }
            } else if (ota_state == ESP_OTA_IMG_VALID) {
                Serial.println("[OTA] Running verified firmware");
            }
        }
    }

#if defined(CAN_DRIVER_TWAI)
  #if defined(BOARD_WAVESHARE_S3)
    Serial.println("[CAN] Driver: ESP32-S3 TWAI (Waveshare ESP32-S3-RS485-CAN)");
  #elif defined(BOARD_LILYGO)
    Serial.println("[CAN] Driver: ESP32 TWAI (LilyGO T-CAN485)");
  #else
    Serial.println("[CAN] Driver: ESP32 TWAI (M5Stack ATOM Lite + ATOMIC CAN Base)");
  #endif
#elif defined(CAN_DRIVER_MCP2515)
    Serial.println("[CAN] Driver: MCP2515 via SPI");
#endif

#if defined(BOARD_LILYGO)
    pinMode(ME2107_EN, OUTPUT);
    digitalWrite(ME2107_EN, HIGH);
    delay(100); // Wait for 5V rail to stabilize (SD power)
    // CAN transceiver slope/mode pin — must be LOW for normal TX+RX operation.
    // Floating or HIGH puts the SN65HVD230/TJA1051 into standby (RX-only),
    // which causes the TWAI controller to go bus-off the first time it tries to TX.
    pinMode(PIN_CAN_SPEED_MODE, OUTPUT);
    digitalWrite(PIN_CAN_SPEED_MODE, LOW);
#endif

    Serial.printf("[CFG] pins: LED=%d BUTTON=%d CAN_TX=%d CAN_RX=%d\n",
                  PIN_LED, PIN_BUTTON, PIN_CAN_TX, PIN_CAN_RX);

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    led_init();

    fsd_state_init(&g_state, TeslaHW_Unknown);
    // Explicit safe defaults — will be overridden after HW auto-detect
    g_state.op_mode               = OpMode_ListenOnly;
    g_state.nag_killer            = true;
    g_state.suppress_speed_chime  = true;
    g_state.emergency_vehicle_detect = false;
    g_state.force_fsd             = false;
    g_state.china_mode            = false;
    g_state.profile_mode_auto     = true;
    g_state.manual_speed_profile  = 1;
    g_state.hw3_offset_auto       = true;
    g_state.hw3_offset            = 0;
    g_state.hw3_offset_percent_mode = false;
    g_state.hw3_offset_active     = 0;
    g_state.hw4_offset            = 0;
    g_state.hw4_offset_percent_mode = false;
    g_state.hw4_offset_active     = 0;
    g_state.das_speed_limit_active = 0;
    g_state.bms_output            = false;

    prefs_load(&g_state);

    {
        esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
        g_factory_reset_window = (wakeup == ESP_SLEEP_WAKEUP_UNDEFINED);
        if (g_factory_reset_window)
            Serial.println("[BTN] Factory reset window active — hold button 5s within 20s");
    }

    if (g_state.op_mode == OpMode_Active) {
        // Will be re-applied after g_can is created; record intent here only
        Serial.println("[NVS] Restored Active mode from NVS");
    }

    led_set(LED_BLUE);

    can_dump_init();

#if defined(BOARD_LILYGO)
    {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_EXT0) {
            Serial.printf("[WAKE] Woken by CAN activity (EXT0 GPIO %d)\n", PIN_CAN_RX);
        } else if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
            Serial.printf("[WAKE] Wakeup cause=%d\n", (int)cause);
        }
        g_last_can_rx_ms = millis();
    }
#endif

    g_can = can_driver_create();
    if (!g_can->begin(true)) {
        Serial.println("[ERR] CAN driver init FAILED — check wiring");
        led_set(LED_RED);
        // Halt: signal error via blinking red indefinitely
        while (true) {
            led_set(LED_RED);   delay(200);
            led_set(LED_OFF);   delay(200);
        }
    }

    if (g_state.op_mode == OpMode_Active) {
        g_can->setListenOnly(false);
        Serial.println("[CAN] 500 kbps — Active (restored from NVS)");
    } else {
        Serial.println("[CAN] 500 kbps — Listen-Only");
    }
    Serial.println("[BTN] Single click : toggle Listen-Only / Active");
    Serial.println("[BTN] Long press 3s: toggle NAG Killer");
    Serial.println("[BTN] Double click : toggle BMS serial output");
    Serial.println("[LED] Blue=Listen  Green=Active  Yellow=OTA  Red=Error");

    // ── WiFi AP + Web dashboard (non-fatal if WiFi fails) ─────────────────────
    if (wifi_ap_init(&g_state)) {
        web_dashboard_init(&g_state, g_can, &g_state_mux);
    }

    // ── Create CAN task pinned to Core 1 ──────────────────────────────────────
    BaseType_t can_task_ok = xTaskCreatePinnedToCore(
        can_task,
        "CANTask",
        8192,
        nullptr,
        3,
        &g_can_task_handle,
        1);
    if (can_task_ok == pdPASS) {
        Serial.println("[CAN] Task created on Core 1");
    } else {
        Serial.println("[ERR] CAN task creation FAILED");
        led_set(LED_RED);
        while (true) {
            led_set(LED_RED); delay(200);
            led_set(LED_OFF); delay(200);
        }
    }
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    vTaskDelay(pdMS_TO_TICKS(100));
}
