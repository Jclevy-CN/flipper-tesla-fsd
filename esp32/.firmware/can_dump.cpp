#include "can_dump.h"

// SD-backed CAN dump support is intentionally disabled on every build,
// including BOARD_LILYGO. Keeping the public API as no-ops lets the rest of
// the firmware share one call path without introducing cross-task SD access.
void   can_dump_init()                    {}
bool   can_dump_start()                   { return false; }
void   can_dump_stop()                    {}
void   can_dump_record(const CanFrame &)  {}
void   can_dump_tick(uint32_t)            {}
bool   can_dump_active()                  { return false; }
void   can_dump_log(const char *, ...)    {}
void   sd_syslog(const char *, ...)       {}
void   sd_syslog_close()                  {}
String sd_format_card()                   { return "{\"ok\":false,\"msg\":\"SD support is disabled\"}"; }
