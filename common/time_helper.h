#pragma once
#include "helper.h"

// This is just to make the code a bit clearer
using u32_micros = u32;
using u64_micros = u64;

void time_init();

// monotonic, platform-independent and precise time helper function
u64_micros time_micros_64();

// This version overflows after 35 minutes, but it's fine for timing fast stuff.
u32_micros time_micros();

// Waiting a precise amount of time, but possibly using more CPU time
void precise_sleep(double seconds);

// More imprecise, but cheap CPU wise
void imprecise_sleep(double seconds);
