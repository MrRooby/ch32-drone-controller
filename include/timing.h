#ifndef TIMING_H
#define TIMING_H
#include "funconfig.h"

#define TICKS_PER_MS (FUNCONF_SYSTEM_CORE_CLOCK / 10000)

uint32_t millis() {
    return funSysTick32() / TICKS_PER_MS;
}
#endif // !TIMING_H
