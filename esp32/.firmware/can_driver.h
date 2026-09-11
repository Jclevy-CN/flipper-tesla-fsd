#pragma once

#include "fsd_handler.h"  // for CanFrame

struct CanErrorStats {
    uint32_t rx_missed;
    uint32_t bus_errors;
    uint32_t rx_overrun;
    uint32_t rx_rejected;
    uint32_t tx_invalid;
    uint32_t rx_pending;
    uint32_t tx_pending;
    uint32_t rx_queue_peak;
    uint32_t tx_queue_peak;
    bool     queue_stats_valid;
    uint8_t  state;
};

enum class CanModeResult : uint8_t {
    Switched,
    SwitchFailedRolledBack,
    SwitchFailedDriverDown
};

typedef bool (*CanTransmitGate)(const CanFrame &frame);

// ── Abstract CAN driver ───────────────────────────────────────────────────────
// Implemented by TwaiDriver (CAN_DRIVER_TWAI) and Mcp2515Driver (CAN_DRIVER_MCP2515).
// Compile-time selection via platformio.ini build_flags.

class CanDriver {
protected:
    CanTransmitGate transmit_gate_ = nullptr;

    bool transmitAllowed(const CanFrame &frame) const {
        return transmit_gate_ == nullptr || transmit_gate_(frame);
    }

public:
    /** Install the final gate checked immediately before hardware TX. */
    void setTransmitGate(CanTransmitGate gate) { transmit_gate_ = gate; }

    /** Initialise hardware and start the CAN bus.
     *  @param listen_only  If true, enter hardware listen-only mode (no ACK, no TX). */
    virtual bool begin(bool listen_only) = 0;

    /** Send one CAN frame.  Returns false when TX is not allowed (listen-only, bus-off, etc.). */
    virtual bool send(const CanFrame &frame) = 0;

    /** Non-blocking receive.  Fills frame and returns true if a frame was available. */
    virtual bool receive(CanFrame &frame) = 0;

    /** Cumulative CAN controller diagnostics. */
    virtual CanErrorStats errorStats() = 0;

    /** Service controller health and recover from bus-off when supported.
     *  Returns true when a recovery completed and the controller restarted. */
    virtual bool serviceHealth() = 0;

    /** Switch between listen-only and normal TX mode at runtime.
     *  Implementations must reinitialise the hardware as needed. */
    virtual CanModeResult setListenOnly(bool enable) = 0;

    /** Restart the CAN controller in the requested mode. */
    virtual bool restart(bool listen_only) = 0;

    /** Discard frames already waiting in the controller's transmit queue. */
    virtual bool clearPendingTransmit() = 0;

    virtual ~CanDriver() = default;
};

/** Factory function — returns the driver selected at compile time.
 *  Caller owns the returned pointer. */
CanDriver *can_driver_create();
