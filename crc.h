#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

uint32_t crc32_update(uint8_t ch, uint32_t crc);
uint32_t crc32(const void *buf, size_t len);
uint32_t crc32_range(const void *from, const void *to);

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
