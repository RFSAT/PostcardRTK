// util.h - debug printing. USB carries the receiver stream when the USB bridge is on,
// so debug text goes to USB only when the bridge is off.
#pragma once
void dbg(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Restart cleanly: stop Bluetooth and Wi-Fi and power the radio chip off first,
// so it starts cold after the reset (a warm radio can stall the next start-up).
void safeReboot() __attribute__((noreturn));
// Safe power-down: close the caster connection, stop the outputs and the receiver link,
// switch the radio off and halt. Then the board can be unplugged.
void safeShutdown() __attribute__((noreturn));
