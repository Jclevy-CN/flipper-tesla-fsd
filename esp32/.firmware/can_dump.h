#pragma once
#include <Arduino.h>
#include "fsd_handler.h"

// Compatibility API for the former SD-card CAN logger. SD support is disabled
// for every target, including LilyGO, so all operations are safe no-ops.

void   can_dump_init();
bool   can_dump_start();
void   can_dump_stop();
void   can_dump_record(const CanFrame &frame);
void   can_dump_tick(uint32_t now_ms);
bool   can_dump_active();
String sd_format_card();

// No-op. Retained so diagnostic call sites do not need board conditionals.
void   can_dump_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// No-op compatibility functions for the former persistent SD system log.
void   sd_syslog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

void   sd_syslog_close();
