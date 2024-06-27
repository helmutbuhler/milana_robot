#include "time_helper.h"

#ifdef _MSC_VER
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#include <sys/resource.h>
#include <sys/param.h>
#endif
#include <cstdlib>
#include <stdio.h>
#include <time.h>


#if defined(_MSC_VER) || defined(__MINGW32__)
static int64_t timer_freq, timer_start;
void time_init()
{
    LARGE_INTEGER t;
    QueryPerformanceFrequency(&t);
    timer_freq = t.QuadPart;

    // The multiplication by 1000 or 1000000 below can cause an overflow if timer_freq
    // and the uptime is high enough.
    // We subtract the program start time to reduce the likelihood of that happening.
    QueryPerformanceCounter(&t);
    timer_start = t.QuadPart;
}

u64_micros time_micros_64()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
	// This isn't quite optimal, we first multiply and then divide. That can overflow.
	// But I didn't find a simple to use MulDiv function for 64 bit ints.
	// And because we subtract timer_start, it shouldn't overflow for days.
    return ((t.QuadPart-timer_start) * 1000000) / timer_freq;
}
#else
void time_init() {}
// Get a time stamp in microseconds.
// source: https://stackoverflow.com/a/67731965
#define SEC_TO_US(sec) ((sec)*1000000)
#define NS_TO_US(ns)   ((ns)/1000)
u64 time_micros_64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    u64 us = SEC_TO_US((u64)ts.tv_sec) + NS_TO_US((u64)ts.tv_nsec);
    return us;
}
#endif

u32_micros time_micros()
{
	return (u32_micros)time_micros_64();
}

void precise_sleep(double seconds)
{
#ifdef _MSC_VER
	// On Windows, Sleep is very imprecise. It can sleep on the order of 20ms
	// longer than specified. To prevent that we sleep the final 20ms by busy waiting
	// with Sleep(0). That won't save CPU time, but at least other threads have the
	// chance to run.
	// I also tried sleeping with CreateWaitableTimer, but that's as inprecise as Sleep.
	if (seconds <= 0)
		return;
	u64 start = time_micros_64();
	u64 micros = u64(seconds*1000000.0);
	if (micros > 40000)
		Sleep((DWORD)((micros-20000)/1000));
	while (time_micros_64()-start < micros)
		Sleep(0);
#else
	// source: https://stackoverflow.com/questions/29243572/how-to-pause-for-a-non-integer-amount-of-time-in-c
    const long sec = (long)seconds;
    const long nsec = (long)((seconds - (double)sec) * 1e9);
    struct timespec  req, rem;

    if (sec < 0L)
        return;
    if (sec == 0L && nsec <= 0L)
        return;

    req.tv_sec = sec;
    if (nsec <= 0L)
        req.tv_nsec = 0L;
    else
    if (nsec <= 999999999L)
        req.tv_nsec = nsec;
    else
        req.tv_nsec = 999999999L;

    rem.tv_sec = 0;
    rem.tv_nsec = 0;

    nanosleep(&req, &rem);
#endif
}

void imprecise_sleep(double seconds)
{
#ifdef _MSC_VER
	if (seconds <= 0)
		return;
	Sleep((DWORD)(seconds*1000.0));
#else
	// todo!
	precise_sleep(seconds);
#endif
}

