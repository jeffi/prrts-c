#ifndef ATOMIC_H
#define ATOMIC_H

#define smp_write_barrier() __sync_synchronize()
#define smp_read_barrier() __sync_synchronize()
#define smp_read_barrier_depends() ((void)0)
#define smp_full_barrier() __sync_synchronize()


/*
 * GCC does not expose a "__sync_get".  One option is to use a
 * __sync_fetch_and_add(ptr, 0), but this is overkill since it will
 * perform a write to the address, screwing up the cache lines just
 * for a semantically correct read.
 *
 * Fortunately, on intel and sparc architectures, "the memory models
 * are strong enough to eliminate the need for a memory barrier.",
 * see: http://www.infoq.com/articles/memory_barriers_jvm_concurrency
 *
 * Unfortunately on other architectures (e.g. itanium), we may need to
 * fallback to the __sync_fetch_and_add strategy.
 */
#if 0
#define __sync_get(ptr) __sync_fetch_and_add(ptr, 0)
#else
#define __sync_get(ptr) (*(ptr))
#endif

#ifdef INTEL
#define __sync_set(ptr, value) __sync_lock_test_and_set(ptr, value)
#else

/*
 * GCC documentation states the __sync_lock_test_and_set builtin is
 * not guaranteed to work on all architectures.  This is a workaround
 * that sets the value locally than updates it atomically.
 */

/*
 * This version of __sync_set may be overkill, but it relies only upon
 * the gcc builtin atomics, and guarantees that the address gets the
 * new value written (at least momentarily).  We also could initialize
 * the loop with a __sync_get, but instead we just try a standard
 * memory access, the compare_and_swap loop that follows will handle a
 * read that returned an inconsistent view of memory.)
 */
#define __sync_set(ptr, value) do {                                     \
                typeof(*ptr) __oldval1__ = (*ptr);                      \
                typeof(*ptr) __oldval2__;                               \
                for (;;) {                                              \
                        if ((__oldval2__ = __sync_val_compare_and_swap((ptr), __oldval1__, (value))) == __oldval1__) \
                                break;                                  \
                        if ((__oldval1__ = __sync_val_compare_and_swap((ptr), __oldval2__, (value))) == __oldval2__) \
                                break;                                  \
                }                                                       \
        } while (0)

#endif /* INTEL */

#endif /* ATOMIC_H */
