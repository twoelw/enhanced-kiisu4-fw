// Simple glue layer for display API
#include "rw_display.h"
#include "rw_sh1106.h"

void rw_display_init(void) { rw_sh1106_init(); }
// The rest is implemented directly in rw_sh1106.c for now
