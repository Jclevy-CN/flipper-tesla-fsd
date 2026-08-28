/*
 * can_driver.cpp
 *
 * CAN driver abstraction: compile-time selection between
 *   CAN_DRIVER_TWAI   — ESP32 built-in TWAI peripheral (M5Stack ATOM Lite + ATOMIC CAN Base)
 *   CAN_DRIVER_MCP2515 — SPI-attached MCP2515 (generic ESP32 boards)
 *
 * Both drivers implement the CanDriver interface from can_driver.h.
 */

#include "can_driver.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

// ── TWAI driver ───────────────────────────────────────────────────────────────
#if defined(CAN_DRIVER_TWAI)

#include "driver/twai.h"

class TwaiDriver : public CanDriver {
    bool     listen_only_ = false;
    bool     installed_   = false;
    bool     recovering_  = false;

    bool install_and_start(bool listen_only) {
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)PIN_CAN_TX,
            (gpio_num_t)PIN_CAN_RX,
            listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
        // Party CAN can exceed 2k fps; keep enough RX headroom for dashboard/dump work.
        g.rx_queue_len = 64;
        g.tx_queue_len = 10;

        twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
        if (twai_start() != ESP_OK) {
            twai_driver_uninstall();
            return false;
        }
        installed_    = true;
        listen_only_  = listen_only;
        recovering_   = false;
        return true;
    }

    void stop_and_uninstall() {
        if (!installed_) return;
        twai_stop();
        twai_driver_uninstall();
        installed_ = false;
        recovering_ = false;
    }

public:
    bool begin(bool listen_only) override {
        return install_and_start(listen_only);
    }

    bool send(const CanFrame &frame) override {
        if (listen_only_) return false;
        twai_message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.identifier       = frame.id;
        msg.data_length_code = frame.dlc;
        memcpy(msg.data, frame.data, frame.dlc);
        // 5 ms TX timeout — short enough to not stall the main loop
        return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
    }

    bool receive(CanFrame &frame) override {
        twai_message_t msg;
        // Non-blocking receive (timeout = 0)
        if (twai_receive(&msg, 0) != ESP_OK) return false;
        frame.id  = msg.identifier;
        frame.dlc = msg.data_length_code;
        memcpy(frame.data, msg.data, frame.dlc);
        return true;
    }

    CanErrorStats errorStats() override {
        twai_status_info_t info;
        CanErrorStats stats = {};
        if (twai_get_status_info(&info) != ESP_OK) return stats;
        stats.rx_missed = info.rx_missed_count;
        stats.bus_errors = info.bus_error_count;
        stats.rx_overrun = info.rx_overrun_count;
        stats.state = (uint8_t)info.state;
        return stats;
    }

    bool serviceHealth() override {
        if (!installed_) return false;

        twai_status_info_t info;
        if (twai_get_status_info(&info) != ESP_OK) return false;

        if (info.state == TWAI_STATE_BUS_OFF && !recovering_) {
            if (twai_initiate_recovery() == ESP_OK) {
                recovering_ = true;
                Serial.println("[CAN] Bus-off detected — initiating recovery");
            }
        } else if (recovering_ && info.state == TWAI_STATE_STOPPED) {
            if (twai_start() == ESP_OK) {
                recovering_ = false;
                Serial.println("[CAN] Bus recovered — restarted");
                return true;
            }
        }
        return false;
    }

    CanModeResult setListenOnly(bool enable) override {
        if (installed_ && listen_only_ == enable) return CanModeResult::Switched;
        bool previous_mode = listen_only_;
        stop_and_uninstall();
        if (install_and_start(enable)) return CanModeResult::Switched;

        // Preserve the last known-good mode when the requested switch fails.
        if (install_and_start(previous_mode)) {
            return CanModeResult::SwitchFailedRolledBack;
        }
        return CanModeResult::SwitchFailedDriverDown;
    }

    bool restart(bool listen_only) override {
        stop_and_uninstall();
        return install_and_start(listen_only);
    }
};

CanDriver *can_driver_create() {
    return new TwaiDriver();
}

// ── MCP2515 driver ────────────────────────────────────────────────────────────
#elif defined(CAN_DRIVER_MCP2515)

#include <SPI.h>
#include <mcp2515.h>   // autowp/autowp-mcp2515

class Mcp2515Driver : public CanDriver {
    MCP2515  mcp_;
    bool     listen_only_  = false;
    bool     ready_        = false;
    uint32_t err_count_    = 0;

public:
    Mcp2515Driver() : mcp_(PIN_MCP_CS) {}

    bool begin(bool listen_only) override {
        ready_ = false;
        SPI.begin(PIN_MCP_SCK, PIN_MCP_MISO, PIN_MCP_MOSI, PIN_MCP_CS);
        SPI.setFrequency(8000000);

        mcp_.reset();
        if (mcp_.setBitrate(CAN_500KBPS, MCP_CRYSTAL_MHZ) != MCP2515::ERROR_OK)
            return false;

        MCP2515::ERROR err = listen_only
            ? mcp_.setListenOnlyMode()
            : mcp_.setNormalMode();
        if (err != MCP2515::ERROR_OK) return false;
        listen_only_ = listen_only;
        ready_ = true;
        return true;
    }

    bool send(const CanFrame &frame) override {
        if (!ready_ || listen_only_) return false;
        struct can_frame f;
        f.can_id  = frame.id;
        f.can_dlc = frame.dlc;
        memcpy(f.data, frame.data, frame.dlc);
        if (mcp_.sendMessage(&f) != MCP2515::ERROR_OK) {
            err_count_++;
            return false;
        }
        return true;
    }

    bool receive(CanFrame &frame) override {
        struct can_frame f;
        if (mcp_.readMessage(&f) != MCP2515::ERROR_OK) return false;
        frame.id  = f.can_id;
        frame.dlc = f.can_dlc;
        memcpy(frame.data, f.data, f.can_dlc);
        return true;
    }

    CanErrorStats errorStats() override {
        CanErrorStats stats = {};
        stats.bus_errors = err_count_;
        return stats;
    }

    bool serviceHealth() override {
        return false;
    }

    CanModeResult setListenOnly(bool enable) override {
        if (ready_ && listen_only_ == enable) return CanModeResult::Switched;
        if (!ready_) {
            return begin(enable) ? CanModeResult::Switched
                                 : CanModeResult::SwitchFailedDriverDown;
        }
        MCP2515::ERROR err = enable ? mcp_.setListenOnlyMode() : mcp_.setNormalMode();
        if (err != MCP2515::ERROR_OK) {
            return CanModeResult::SwitchFailedRolledBack;
        }
        listen_only_ = enable;
        return CanModeResult::Switched;
    }

    bool restart(bool listen_only) override {
        return begin(listen_only);
    }
};

CanDriver *can_driver_create() {
    return new Mcp2515Driver();
}

#else
#error "Define CAN_DRIVER_TWAI or CAN_DRIVER_MCP2515 in platformio.ini build_flags"
#endif
