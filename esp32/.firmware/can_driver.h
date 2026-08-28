#pragma once

#include "fsd_handler.h"  // for CanFrame

struct CanErrorStats {
    uint32_t rx_missed;
    uint32_t bus_errors;
    uint32_t rx_overrun;
    uint8_t  state;
};

// ── Abstract CAN driver ───────────────────────────────────────────────────────
// Implemented by TwaiDriver (CAN_DRIVER_TWAI) and Mcp2515Driver (CAN_DRIVER_MCP2515).
// Compile-time selection via platformio.ini build_flags.

class CanDriver {
public:
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
    virtual bool setListenOnly(bool enable) = 0;

    /** Restart the CAN controller in the requested mode. */
    virtual bool restart(bool listen_only) = 0;

    virtual ~CanDriver() = default;
};

/** Factory function — returns the driver selected at compile time.
 *  Caller owns the returned pointer. */
CanDriver *can_driver_create();
