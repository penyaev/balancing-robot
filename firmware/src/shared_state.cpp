// shared_state.cpp — see shared_state.h.

#include "shared_state.h"

#include <Arduino.h>

namespace shared {

SharedState g;

void setFlag(Flag f, bool on) {
  uint16_t cur = g.flags.load(std::memory_order_relaxed);
  uint16_t desired;
  do {
    desired = on ? (cur | static_cast<uint16_t>(f))
                 : (cur & static_cast<uint16_t>(~f));
    if (desired == cur) return;
  } while (!g.flags.compare_exchange_weak(cur, desired,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed));
}

bool getFlag(Flag f) {
  return (g.flags.load(std::memory_order_acquire) &
          static_cast<uint16_t>(f)) != 0;
}

void printFlags(uint16_t bits, ::Print& out) {
  bool any = false;
  auto emit = [&](const __FlashStringHelper* s) {
    if (any) out.print('|');
    out.print(s);
    any = true;
  };
  if (bits & FLAG_FALLEN)         emit(F("FALLEN"));
  if (bits & FLAG_LOW_BAT)        emit(F("LOW_BAT"));
  if (bits & FLAG_MOTORS_ENABLED) emit(F("MOTORS"));
  if (bits & FLAG_PS_CONNECTED)   emit(F("PS"));
  if (!any) out.print(F("none"));
}

} // namespace shared
