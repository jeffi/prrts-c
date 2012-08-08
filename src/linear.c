#include <math.h>
#include <string.h>
#include <stdio.h>
#include "linear.h"

/* Include with inline functions to create single function addresses */
#define INLINE
#include "linear_inline.h"
#undef INLINE

const mat4_t M4_IDENTITY = {
        {
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
        }
};

double
v3_dist_segment_point(const vec3_t *s0, const vec3_t *s1, const vec3_t *pt)
{
        vec3_t v, w;
        double c1, c2, b;

        v3_sub(&v, s1, s0);
        v3_sub(&w, pt, s0);

        c1 = v3_dot(&w, &v);
        if (c1 <= 0.0) {
                return v3_len(&w);
        }

        c2 = v3_dot(&v, &v);
        if (c2 <= c1) {
                return v3_dist(pt, s1);
        }

        b = c1 / c2;

        v3_scale(&v, &v, b);
        v3_add(&v, s0, &v);

        return v3_dist(&v, pt);
}

void
m4_print(const mat4_t *m)
{
        printf("[ %f %f %f %f "
               "/ %f %f %f %f "
               "/ %f %f %f %f "
               "/ %f %f %f %f ]",

               m->m00, m->m01, m->m02, m->m03,
               m->m10, m->m11, m->m12, m->m13,
               m->m20, m->m21, m->m22, m->m23,
               m->m30, m->m31, m->m32, m->m33);
}
