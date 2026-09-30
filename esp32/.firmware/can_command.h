#pragma once

#include <Arduino.h>
#include "fsd_handler.h"

enum class CanCommandType : uint8_t {
    SetMode,
    ShutdownForRestart
};

struct CanCommand {
    CanCommandType type;
    OpMode requested_mode;
    uint32_t request_id;
    TaskHandle_t reply_task;
};
