#ifndef LINEAR_H
#define LINEAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

typedef union {
        struct {
                double m00, m01, m02, m03;
                double m10, m11, m12, m13;
                double m20, m21, m22, m23;
                double m30, m31, m32, m33;
        };
        double m[16];
} mat4_t;

typedef union {
        struct {
                double x, y;
        };
        double v[2];
} vec2_t;

typedef union {
        struct {
                double x, y, z;
        };
        double v[3];
        vec2_t v2;
} vec3_t;

typedef union {
        struct {
                double x, y, z, w;
        };
        double v[4];
        vec2_t v2;
        vec3_t v3;
} vec4_t;


#if __GNUC__ && !__GNUC_STD_INLINE__
#  define INLINE extern inline
#else
#  define INLINE inline
#endif

#include "linear_inline.h"

extern const mat4_t M4_IDENTITY;

double v3_dist_segment_point(const vec3_t *s0, const vec3_t *s1, const vec3_t *pt);
void m4_print(const mat4_t *m);

#undef INLINE


#ifdef __cplusplus
}
#endif

#endif /* LINEAR_H */
