#ifndef HRTIMER_H
#define HRTIMER_H

#include <stdint.h>

typedef int64_t hrtimer_t;

#define HRTIMER_MAX INT64_MAX
#define HRTIMER_MIN INT64_MIN

hrtimer_t hrtimer_get();

/* resolution of the hrtimer (nanoseconds) */
#define HRTICK 1000000000

#endif /* HRTIMER_H */
