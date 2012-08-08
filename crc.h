#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

extern const uint32_t crc32_table[];

typedef uint32_t crc32_t;

#define CRC32_INIT 0xFFFFFFFF

static inline crc32_t
crc32_update(uint8_t ch, crc32_t crc) {
        return crc32_table[(crc ^ ch) & 0xff] ^ (crc >> 8);
}

static inline crc32_t
crc32_finish(crc32_t crc) {
        return ~crc;
}

static inline void
crc32_accum(crc32_t *accum, const void *buf, size_t len) {
        const uint8_t *ptr = buf;
        unsigned i;

        for (i=0 ; i<len ; ++i) {
                *accum = crc32_update(ptr[i], *accum);
        }
}

static inline crc32_t
crc32(const void *buf, size_t len) {
        crc32_t accum = CRC32_INIT;
        crc32_accum(&accum, buf, len);
        return crc32_finish(accum);
}

#ifdef CHECK_CRCS

/*
 * computes the crc32 of a structure and records it in the crc32
 * field.
 */
#define CRC32_SET(ptr, from, to) (ptr)->crc32 = crc32_range(&(ptr)->from, &(ptr)->to)

/*
 * parallel to CRC32_SET, computes and checks the value of the crc32
 * field.
 */
#define CRC32_CHECK(ptr, from, to)  assert((ptr)->crc32 == crc32_range(&(ptr)->from, &(ptr)->to))

#else

#define CRC32_SET(ptr, from, to) ((void)0)
#define CRC32_CHECK(ptr, from, to) ((void)0)

#endif


#endif /* CRC32_H */
