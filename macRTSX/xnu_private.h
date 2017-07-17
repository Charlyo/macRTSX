/*
 * These functions seem to be unsupported, but still accessible
 * using the "com.apple.kpi.unsupported" bundle library.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

extern int tsleep(void *chan, int pri, const char *wmesg, int timo);

#ifdef __cplusplus
}
#endif
