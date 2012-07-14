#ifndef COLLIDE_H
#define COLLIDE_H

#include <stdbool.h>
#include "linear.h"

extern int collide_trace;

bool collide_sphere_capsule(const mat4_t *sphere_transform, double sphere_radius,
                            const mat4_t *capsule_transform, double capsule_length, double capsule_radius);

bool collide_sphere_sphere(const mat4_t *a_transform, double a_radius,
                           const mat4_t *b_transform, double b_radius);

bool collide_capsule_capsule(const mat4_t *a_transform, double a_length, double a_radius,
                             const mat4_t *b_transform, double b_length, double b_radius);


#endif /* COLLIDE_H */
