#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "linear.h"
#include "collide.h"

int collide_trace = 0;

bool
collide_sphere_capsule(const mat4_t *sphere_transform, double sphere_radius,
                       const mat4_t *capsule_transform, double capsule_length, double capsule_radius)
{
        vec3_t sphere_center;
        vec3_t capsule_start;
        vec3_t capsule_end;
        double dist;

        m4_transform_i3(&sphere_center, sphere_transform, 0.0, 0.0, 0.0);
        m4_transform_i3(&capsule_start, capsule_transform, 0.0, 0.0, 0.0);
        m4_transform_i3(&capsule_end, capsule_transform, 0.0, 0.0, capsule_length);

        if (collide_trace) {
                printf("capsule (%f, %f, %f) to (%f, %f, %f) radius=%f, sphere (%f, %f, %f) radius=%f\n",
                       capsule_start.x, capsule_start.y, capsule_start.z,
                       capsule_end.x, capsule_end.y, capsule_end.z, capsule_radius,
                       sphere_center.x, sphere_center.y, sphere_center.z, sphere_radius);
        }

        dist = v3_dist_segment_point(&capsule_start, &capsule_end, &sphere_center);

        return dist < (sphere_radius + capsule_radius);
}

bool
collide_sphere_sphere(const mat4_t *a_transform, double a_radius,
                      const mat4_t *b_transform, double b_radius)
{
        vec4_t a_center, b_center;
        double d2, r2;

        m4_transform_i(&a_center, a_transform, 0.0, 0.0, 0.0, 1.0);
        m4_transform_i(&b_center, b_transform, 0.0, 0.0, 0.0, 1.0);

        d2 = v3_dist2(&a_center.v3, &b_center.v3);
        r2 = a_radius + b_radius;
        r2 *= r2;

        return (d2 < r2);
}

bool
collide_capsule_capsule(const mat4_t *a_transform, double a_length, double a_radius,
                        const mat4_t *b_transform, double b_length, double b_radius)
{
        vec4_t a_s0, a_s1;
        vec4_t b_s0, b_s1;
        double a0, a1;
        double b0, b1;
        double dist;

        m4_transform_i(&a_s0, a_transform, 0.0, 0.0, 0.0, 1.0);
        m4_transform_i(&a_s1, a_transform, 0.0, 0.0, a_length, 1.0);

        m4_transform_i(&b_s0, b_transform, 0.0, 0.0, 0.0, 1.0);
        m4_transform_i(&b_s1, b_transform, 0.0, 0.0, b_length, 1.0);

        a0 = v3_dist_segment_point(&a_s0.v3, &a_s1.v3, &b_s0.v3);
        a1 = v3_dist_segment_point(&a_s0.v3, &a_s1.v3, &b_s1.v3);

        b0 = v3_dist_segment_point(&b_s0.v3, &b_s1.v3, &a_s0.v3);
        b1 = v3_dist_segment_point(&b_s0.v3, &b_s1.v3, &a_s1.v3);

        dist = fmin(fmin(a0, a1), fmin(b0, b1));

        return dist < (a_radius + b_radius);
        
}
