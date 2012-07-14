#include <sys/time.h>
#include <stdlib.h>
#include <err.h>
#include <time.h>
#include "hrtimer.h"

hrtimer_t
hrtimer_get()
{
#ifdef CLOCK_REALTIME
        struct timespec now;
        hrtimer_t t;

        if (clock_gettime(CLOCK_REALTIME, &now) != 0) {
                warn("clock_gettime failed");
                return -1;
        }

        t = now.tv_sec;
        t *= 1000000000L;
        t += now.tv_nsec;
        return t;
#else
        struct timeval now;
        hrtimer_t t;

        if (gettimeofday(&now, NULL) == -1) {
                warn("gettimeofday failed");
                return -1;
        }

        t = now.tv_sec;
        t *= 1000000000L;
        t += now.tv_usec * 1000L;
        return t;
#endif
}

