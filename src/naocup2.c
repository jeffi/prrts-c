#include <math.h>
#include <assert.h>
#include <err.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "linear.h"
#include "collide.h"
#include "alloc.h"
#include "prrts.h"
#include "hrtimer.h"

#define DIMENSIONS 10
#define INCHES 0.0254

/* cos(25 degrees) */
#define RHAND_UP_GOAL 0.90630778703665

/* cos(45 degrees) */
#define LHAND_UP_GOAL 0.707106781186548

#define CENTER_TORSO_RADIUS (66.7 / 1000.0)
#define CUP_BEAD_COUNT 8
#define CUP_BEAD_RADIUS (2.0/4.0 * INCHES)
#define CUP_DIAMETER (2.5 * INCHES)
#define CUP_HEIGHT ((4.0 + 3.0/8.0) * INCHES)
#define CUP_GRIP_HEIGHT (1.0 * INCHES)
#define CUP_BASE_TO_BOWL ((1.0 + 5.0/8.0) * INCHES)
#define CUP_GRIP_DIAMETER (5.0/8.0 * INCHES)
#define CUP_GRIP_CAPSULE_HEIGHT (CUP_BASE_TO_BOWL - CUP_GRIP_DIAMETER*2.0)
#define CUP_BOWL_HEIGHT (CUP_HEIGHT - CUP_BASE_TO_BOWL)

#define BALL_RADIUS 0.015

#define NUM_OBSTACLES 4
#define PLANAR_SPHERE_RADIUS 25.0
#define TABLE_OFFSET_Z 0.09
#define DISCRETIZATION (1.0 * M_PI / 180.0)

typedef struct nao_arm {
        /* joint position */
        double shoulder_pitch;
        double shoulder_roll;
        double elbow_yaw;
        double elbow_roll;
        double wrist_yaw;

        /* computed */
        mat4_t transform_to_arm;

        mat4_t upper_capsule_transform;
        mat4_t lower_capsule_transform;
        mat4_t transform_to_hand;

        mat4_t shoulder_pitch_matrix;
        mat4_t shoulder_roll_matrix;
        mat4_t translate_to_elbow;
        mat4_t elbow_yaw_matrix;
        mat4_t elbow_roll_matrix;
        mat4_t wrist_yaw_matrix;
        mat4_t translate_to_hand;
        mat4_t rotate_hand;
} nao_arm_t;

typedef struct nao_leg {
        double hip_roll;
        double hip_pitch;
        double knee_pitch;
        double ankle_pitch;
        double ankle_roll;
} nao_leg_t;

typedef struct nao_robot {
        mat4_t transform;

        double head_yaw;
        double head_pitch;
        struct nao_arm left_arm;
        struct nao_arm right_arm;
        double hip_yaw_pitch;
        struct nao_leg left_leg;
        struct nao_leg right_leg;

        mat4_t head_yaw_transform;
        mat4_t head_pitch_transform;
        mat4_t head_capsule_rotate;
        mat4_t head_center;
        mat4_t head_capsule;
        /* mat4_t center_torso; */
        mat4_t upper_torso;
        mat4_t lower_torso;

        bool in_self_collision;
} nao_robot_t;

typedef struct nao_world {
        struct nao_robot robot;

        vec3_t cup_up_norm;

        bool in_collision;
        bool in_goal;
        bool cup_is_up;
        bool is_clear;
        double dist_to_goal;
} nao_world_t;

typedef enum nao_joint {
        HEAD_YAW,
        HEAD_PITCH,

        L_SHOULDER_PITCH,
        L_SHOULDER_ROLL,
        L_ELBOW_YAW,
        L_ELBOW_ROLL,
        L_WRIST_YAW,

        R_SHOULDER_PITCH,
        R_SHOULDER_ROLL,
        R_ELBOW_YAW,
        R_ELBOW_ROLL,
        R_WRIST_YAW,

} nao_joint_t;

#define RAD(x)  ((x) * (M_PI / 180.0))
        
#define HEAD_YAW_MIN           RAD(-119.5)
#define HEAD_YAW_MAX           RAD( 119.5)
#define HEAD_PITCH_MIN         RAD( -38.5)
#define HEAD_PITCH_MAX         RAD(  29.5)

#define L_SHOULDER_PITCH_MIN   RAD(-119.5)
#define L_SHOULDER_PITCH_MAX   RAD( 119.5)
#define L_SHOULDER_ROLL_MIN    RAD(   0.5)
#define L_SHOULDER_ROLL_MAX    RAD(  94.5)
#define L_ELBOW_YAW_MIN        RAD(-119.5)
#define L_ELBOW_YAW_MAX        RAD( 119.5)
#define L_ELBOW_ROLL_MIN       RAD( -89.5)
#define L_ELBOW_ROLL_MAX       RAD(  -0.5)
#define L_WRIST_YAW_MIN        RAD(-104.5)
#define L_WRIST_YAW_MAX        RAD( 104.5)

#define R_SHOULDER_PITCH_MIN   RAD(-119.5)
#define R_SHOULDER_PITCH_MAX   RAD( 119.5)
#define R_SHOULDER_ROLL_MIN    RAD( -94.5)
#define R_SHOULDER_ROLL_MAX    RAD(  -0.5)
#define R_ELBOW_YAW_MIN        RAD(-119.5)
#define R_ELBOW_YAW_MAX        RAD( 119.5)
#define R_ELBOW_ROLL_MIN       RAD(   0.5)
#define R_ELBOW_ROLL_MAX       RAD(  89.5)
#define R_WRIST_YAW_MIN        RAD(-104.5)
#define R_WRIST_YAW_MAX        RAD( 104.5)

/* These are from the spec sheets */
#define NECK_OFFSET_Z (126.50 / 1000.0)
#define SHOULDER_OFFSET_Y (98.00 / 1000.0)
#define UPPER_ARM_LENGTH (90.00 / 1000.0)
#define LOWER_ARM_LENGTH (50.55 / 1000.0)
#define SHOULDER_OFFSET_Z (100.00 / 1000.0)
#define HAND_OFFSET_X (58.00 / 1000.0)
#define HIP_OFFSET_Z (85.00 / 1000.0)
#define HIP_OFFSET_Y (50.00 / 1000.0)
#define THIGH_LENGTH (100.00 / 1000.0)
#define TIBIA_LENGTH (102.74 / 1000.0)
#define FOOT_HEIGHT (45.11 / 1000.0)
#define HAND_OFFSET_Z (15.90 / 1000.0)

/* These are made up and need verification */
#define FOOT_WIDTH (90.0 / 1000.0)
#define FOOT_LENGTH (150.0 / 1000.0)
#define FOOT_OFFSET_X (25.0 / 1000.0)
#define FOOT_OFFSET_Y (10.0 / 1000.0)
#define FOOT_OFFSET_Z (FOOT_HEIGHT / 2.0)

#define HEAD_RADIUS (115.0/2.0 / 1000.0)
#define EAR_RADIUS (90.0/2.0 / 1000.0)
#define HEAD_WIDTH (133.0 / 1000.0)
#define TORSO_RADIUS (75.0 / 1000.0)
#define ARM_RADIUS (66.7/2.0 / 1000.0)
#define LEG_RADIUS (88.9/2.0 / 1000.0)
#define HAND_RADIUS (20.0 / 1000.0)
#define HAND_WIDTH (50.0 / 1000.0)

struct collision_object {
        enum {
                SPHERE, CAPSULE
        } type;
        mat4_t *transform;
        double radius;
        double length;
};

#define check_angle(joint, angle) assert((joint##_MIN) <= (angle) && (angle) <= (joint##_MAX));

static const double nao_init_config[] = {
        1.125998,
        -0.691876,
        1.888312,
        0.776246,
        0.245398,

        1.259372,
        0.279146,
        -1.587732,
        -0.510780,
        -1.823800,
};

static const double nao_min_config[] = {
        R_SHOULDER_PITCH_MIN,
        R_SHOULDER_ROLL_MIN,
        R_ELBOW_YAW_MIN,
        R_ELBOW_ROLL_MIN,
        R_WRIST_YAW_MIN,
        L_SHOULDER_PITCH_MIN,
        L_SHOULDER_ROLL_MIN,
        L_ELBOW_YAW_MIN,
        L_ELBOW_ROLL_MIN,
        L_WRIST_YAW_MIN,
};

static const double nao_max_config[] = {
        R_SHOULDER_PITCH_MAX,
        R_SHOULDER_ROLL_MAX,
        R_ELBOW_YAW_MAX,
        R_ELBOW_ROLL_MAX,
        R_WRIST_YAW_MAX,
        L_SHOULDER_PITCH_MAX,
        L_SHOULDER_ROLL_MAX,
        L_ELBOW_YAW_MAX,
        L_ELBOW_ROLL_MAX,
        L_WRIST_YAW_MAX,
};

static const double nao_target_config[] = {
        0.258284303377494,
        -0.2699099199363406,
        -0.01113121187052224,
        1.2053012757652763,
        1.2716626717484503,
        -0.9826967097045605,
        0.07355836822937814,
        0.25450053440459897,
        -0.9512909033938429,
        -0.5297424293532234,
};

static bool
collide_objects(struct collision_object *a, struct collision_object *b)
{
        switch ((a->type << 1) | b->type) {
        case (SPHERE<<1) | SPHERE:
                return collide_sphere_sphere(
                        a->transform, a->radius,
                        b->transform, b->radius);

        case (SPHERE<<1) | CAPSULE:
                return collide_sphere_capsule(
                        a->transform, a->radius,
                        b->transform, b->length, b->radius);

        case (CAPSULE<<1) | SPHERE:
                return collide_sphere_capsule(
                        b->transform, b->radius,
                        a->transform, a->length, a->radius);

        case (CAPSULE<<1) | CAPSULE:
                return collide_capsule_capsule(
                        a->transform, a->length, a->radius,
                        b->transform, b->length, b->radius);

        default:
                assert(0);
        }
}

static bool
collide_object_lists(struct collision_object *a, size_t n,
                     struct collision_object *b, size_t m)
{
        unsigned i, j;
        for (i=0 ; i<n ; ++i) {
                for (j=0 ; j<m ; ++j) {
                        if (collide_objects(a + i, b + j)) {
                                return true;
                        }
                }
        }
        return false;
}

static void
compute_head(mat4_t *transform, nao_robot_t *robot)
{
        m4_rotate(&robot->head_yaw_transform, transform, robot->head_yaw, 0, 0, 1);
        m4_rotate(&robot->head_pitch_transform, &robot->head_yaw_transform, robot->head_pitch, 0, 1, 0);
        m4_translate(&robot->head_center, &robot->head_pitch_transform, 0, 0, HEAD_RADIUS);

        // head_sphere
        //   center = head_center
        //   radius: HEAD_RADIUS

        m4_rotate(&robot->head_capsule_rotate, &robot->head_center, M_PI/2.0, 1, 0, 0);
        m4_translate(&robot->head_capsule, &robot->head_capsule_rotate,
                     0, 0, -(HEAD_WIDTH - EAR_RADIUS*2.0)/2.0);

        // head_capsule
        //   center = head_capsule_transform
        //   radius: EAR_RADIUS
        //   height: HEAD_WIDTH - EAR_RADIUS*2.0
}

static void
compute_arm(nao_arm_t *arm)
{
        m4_rotate(&arm->shoulder_pitch_matrix, &arm->transform_to_arm, arm->shoulder_pitch, 0.0, 1.0, 0.0);
        m4_rotate(&arm->shoulder_roll_matrix, &arm->shoulder_pitch_matrix, arm->shoulder_roll, 0.0, 0.0, 1.0);
        m4_rotate(&arm->upper_capsule_transform, &arm->shoulder_roll_matrix, M_PI/2.0, 0.0, 1.0, 0.0);
        m4_translate(&arm->translate_to_elbow, &arm->shoulder_roll_matrix, UPPER_ARM_LENGTH, 0.0, 0.0);
        m4_rotate(&arm->elbow_yaw_matrix, &arm->translate_to_elbow, arm->elbow_yaw, 1.0, 0.0, 0.0);
        m4_rotate(&arm->elbow_roll_matrix, &arm->elbow_yaw_matrix, arm->elbow_roll, 0.0, 0.0, 1.0);
        m4_rotate(&arm->lower_capsule_transform, &arm->elbow_roll_matrix, M_PI/2.0, 0.0, 1.0, 0.0);
        m4_rotate(&arm->wrist_yaw_matrix, &arm->elbow_roll_matrix, arm->wrist_yaw, 1.0, 0.0, 0.0);

        /*
         * the 0.01 added below is to account for the object's offset
         * within the grasp. it should probably be moved to a constant
         * somewhere else
         */
        m4_translate(&arm->translate_to_hand, &arm->wrist_yaw_matrix,
                     LOWER_ARM_LENGTH + HAND_OFFSET_X + 0.01, 0.0, -HAND_OFFSET_Z - 0.01);

        m4_rotate(&arm->rotate_hand, &arm->translate_to_hand, -M_PI/2.0, 1.0, 0.0, 0.0);
        m4_translate(&arm->transform_to_hand, &arm->rotate_hand, 0.0, 0.0, (HAND_WIDTH - HAND_RADIUS*2.0)/2.0);
}

#if 0
static void
compute_leg(mat4_t *transform, nao_leg_t *leg)
{
        mat4_t  hip_roll_matrix,
                hip_pitch_matrix,
                translate_to_knee,
                knee_matrix,
                translate_to_ankle,
                ankle_pitch_matrix,
                ankle_roll_matrix;

        m4_rotate(&hip_roll_matrix, transform, leg->hip_roll, 1.0, 0.0, 0.0);
        m4_rotate(&hip_pitch_matrix, &hip_roll_matrix, leg->hip_pitch, 0.0, 1.0, 0.0);
        
        m4_translate(&translate_to_knee, &hip_pitch_matrix, 0.0, 0.0, -THIGH_LENGTH);

        m4_rotate(&knee_matrix, &translate_to_knee, leg->knee_pitch, 0.0, 1.0, 0.0);
        m4_translate(&translate_to_ankle, &knee_matrix, 0.0, 0.0, -TIBIA_LENGTH);
        m4_rotate(&ankle_pitch_matrix, &translate_to_ankle, leg->ankle_pitch, 0.0, 1.0, 0.0);
        m4_rotate(&ankle_roll_matrix, &ankle_pitch_matrix, leg->ankle_roll, 1.0, 0.0, 0.0);
}
#endif

static bool
dbool(char *name, bool value)
{
        /* if (value) printf("%s: %s\n", name, value ? "true" : "false"); */
        return value;
}

static void
check_collisions(nao_world_t *world)
{
        struct collision_object left_arm[3];
        struct collision_object right_arm[3];
        struct collision_object torso[3];
        struct collision_object head[2];
        struct collision_object cup[CUP_BEAD_COUNT * 2 + 2];
        struct collision_object obstacles[NUM_OBSTACLES];
        struct collision_object ball[1];
        mat4_t coke_transform;
        mat4_t pepsi_transform;
        mat4_t table_transform;
        mat4_t backwall_transform;
        mat4_t ball_transform;
        mat4_t cup_stem_transform;
        mat4_t cup_bowl_transform;
        mat4_t cup_bead_transform[CUP_BEAD_COUNT * 2];
        int i;

        right_arm[0].type = CAPSULE;
        right_arm[0].transform = &world->robot.right_arm.lower_capsule_transform;
        right_arm[0].radius = ARM_RADIUS;
        right_arm[0].length = LOWER_ARM_LENGTH + HAND_OFFSET_X - ARM_RADIUS;

        left_arm[0].type = CAPSULE;
        left_arm[0].transform = &world->robot.left_arm.lower_capsule_transform;
        left_arm[0].radius = ARM_RADIUS;
        left_arm[0].length = LOWER_ARM_LENGTH + HAND_OFFSET_X - ARM_RADIUS;

        torso[0].type = SPHERE;
        torso[0].transform = &world->robot.transform; /* center_torso */
        torso[0].radius = 55.6/1000.0;
        
        torso[1].type = SPHERE;
        torso[1].transform = &world->robot.upper_torso;
        torso[1].radius = CENTER_TORSO_RADIUS;

        torso[2].type = SPHERE;
        torso[2].transform = &world->robot.lower_torso;
        torso[2].radius = (HIP_OFFSET_Z/2.0);

        head[0].type = SPHERE;
        head[0].transform = &world->robot.head_center;
        head[0].radius = HEAD_RADIUS;

        head[1].type = CAPSULE;
        head[1].transform = &world->robot.head_capsule;
        head[1].radius = EAR_RADIUS;
        head[1].length = HEAD_WIDTH - EAR_RADIUS/2.0;

        // 12oz coke bottle
        m4_translate(&coke_transform, &world->robot.transform, 0.12, 0.08, -TABLE_OFFSET_Z);
        obstacles[0].type = CAPSULE;
        obstacles[0].transform = &coke_transform;
        obstacles[0].radius = 2.5/2.0 * INCHES;
        obstacles[0].length = (6.75 - 2.5/2.0) * INCHES;

        // 20oz pepsi near right hand
        m4_translate(&pepsi_transform, &world->robot.transform, 0.12 + 2.5 * INCHES, -0.12, -TABLE_OFFSET_Z);
        obstacles[1].type = CAPSULE;
        obstacles[1].transform = &pepsi_transform;
        obstacles[1].radius = 3.0/2.0 * INCHES;
        obstacles[1].length = (8.5 - 3.0/2.0) * INCHES;

        // table
        m4_translate(&table_transform, &world->robot.transform, 0.0, 0.0, -PLANAR_SPHERE_RADIUS - TABLE_OFFSET_Z);
        obstacles[2].type = SPHERE;
        obstacles[2].transform = &table_transform;
        obstacles[2].radius = PLANAR_SPHERE_RADIUS;

        // back wall
        m4_translate(&backwall_transform, &world->robot.transform, -PLANAR_SPHERE_RADIUS - CENTER_TORSO_RADIUS, 0.0, 0.0);
        obstacles[3].type = SPHERE;
        obstacles[3].transform = &backwall_transform;
        obstacles[3].radius = PLANAR_SPHERE_RADIUS;

        // ball in hand
        m4_translate(&ball_transform, &world->robot.left_arm.transform_to_hand, 0.0, BALL_RADIUS / 2.0 + 0.01, 0.0);
        ball[0].type = SPHERE;
        ball[0].transform = &ball_transform;
        ball[0].radius = BALL_RADIUS;

        // cup stem
        m4_translate(&cup_stem_transform, &world->robot.right_arm.transform_to_hand,
                     0.0, 0.0, CUP_GRIP_HEIGHT / 2.0);
        cup[0].type = CAPSULE;
        cup[0].transform = &cup_stem_transform;
        cup[0].radius = CUP_GRIP_DIAMETER / 2.0;
        cup[0].length = -CUP_GRIP_CAPSULE_HEIGHT;
        
        // cup bowl
        m4_translate(&cup_bowl_transform, &world->robot.right_arm.transform_to_hand,
                     0.0, 0.0, CUP_GRIP_HEIGHT / 2.0 + CUP_DIAMETER / 2.0);
        cup[1].type = CAPSULE;
        cup[1].transform = &cup_bowl_transform;
        cup[1].radius = CUP_DIAMETER / 2.0;
        cup[1].length = CUP_BOWL_HEIGHT - CUP_DIAMETER;

        // cup beads (representing base and cap)
        for (i=0 ; i<CUP_BEAD_COUNT ; ++i) {
                double a = M_PI * 2.0 * (double)i / (double)CUP_BEAD_COUNT;
                double x = cos(a) * (CUP_DIAMETER / 2.0 - CUP_BEAD_RADIUS);
                double y = sin(a) * (CUP_DIAMETER / 2.0 - CUP_BEAD_RADIUS);
                m4_translate(&cup_bead_transform[i*2], &world->robot.right_arm.transform_to_hand,
                             x, y, CUP_BOWL_HEIGHT + CUP_GRIP_HEIGHT / 2.0 - CUP_BEAD_RADIUS);
                cup[i*2+2].type = SPHERE;
                cup[i*2+2].transform = &cup_bead_transform[i*2];
                cup[i*2+2].radius = CUP_BEAD_RADIUS;

                m4_translate(&cup_bead_transform[i*2+1], &world->robot.right_arm.transform_to_hand,
                             x, y, CUP_GRIP_HEIGHT / 2.0 - CUP_BASE_TO_BOWL + CUP_BEAD_RADIUS);
                cup[i*2+3].type = SPHERE;
                cup[i*2+3].transform = &cup_bead_transform[i*2+1];
                cup[i*2+3].radius = CUP_BEAD_RADIUS;
        }

        // self-collision defined as:
        //   torso-cells(3) & [ left-arm-cells, right-arm-cells ]
        //   left-arm-cells & right-arm-cells
        //   head-cells & [ left-arm-cells, right-arm-cells ]

        // other-collisions
        //   obstacles & [ left-arm-cells, right-arm-cells ]
        //   cup  & [ left-arm-cells, head-cells, torso-cells, obstacles ]
        //   ball & [ right-arm-cells, head-cells, torso-cells, obstacles ]

        world->robot.in_self_collision =
                dbool("torso<->right arm", collide_object_lists(torso, 3, right_arm, 1))
                || dbool("torso<->left arm", collide_object_lists(torso, 3, left_arm, 1))
                || dbool("left<->right arms", collide_object_lists(left_arm, 1, right_arm, 1))
                || dbool("head<->right arm", collide_object_lists(head, 2, right_arm, 1))
                || dbool("head<->left arm", collide_object_lists(head, 2, left_arm, 1));

        world->in_collision = world->robot.in_self_collision
                || dbool("right arm<->obstacles", collide_object_lists(right_arm, 1, obstacles, NUM_OBSTACLES))
                || dbool("left arm<->obstacles", collide_object_lists(left_arm, 1, obstacles, NUM_OBSTACLES))
                || dbool("cup<->torso", collide_object_lists(cup, CUP_BEAD_COUNT * 2 + 2, torso, 3))
                || dbool("cup<->obstacles", collide_object_lists(cup, CUP_BEAD_COUNT * 2 + 2, obstacles, NUM_OBSTACLES))
                || dbool("cup<->left arm", collide_object_lists(cup, CUP_BEAD_COUNT * 2 + 2, left_arm, 1))
                || dbool("cup<->head", collide_object_lists(cup, CUP_BEAD_COUNT * 2 + 2, head, 2))
                || collide_object_lists(right_arm, 1, ball, 1)
                || collide_object_lists(head, 2, ball, 1)
                || collide_object_lists(torso, 3, ball, 1)
                || collide_object_lists(obstacles, NUM_OBSTACLES, ball, 1);
}

static void
compute(nao_world_t *world)
{
#if 0
        mat4_t right_hip_offset;
        mat4_t right_leg_transform;
        mat4_t left_hip_offset;
        mat4_t left_leg_transform;
#endif
        mat4_t head_transform;
        vec4_t r_pos;
        vec4_t cup_bowl_center;
        vec4_t l_pos;
        vec3_t cup_up;
        vec3_t cup_up_norm;
        vec4_t left_hand_ball_down;
        vec3_t ball_down;
        vec3_t ball_down_norm;
        bool cup_is_up;

        world->robot.transform = M4_IDENTITY;

        m4_translate(&head_transform, &world->robot.transform, 0, 0, NECK_OFFSET_Z);
        compute_head(&head_transform, &world->robot);

        /* robot->center_torso = robot->transform; */
        m4_translate(&world->robot.upper_torso, &world->robot.transform, 0, 0, NECK_OFFSET_Z - 66.7 / 1000.0);
        m4_translate(&world->robot.lower_torso, &world->robot.transform, 0, 0, -HIP_OFFSET_Z / 2.0);
        
        m4_translate(&world->robot.right_arm.transform_to_arm, &world->robot.transform, 0, -SHOULDER_OFFSET_Y, SHOULDER_OFFSET_Z);
        compute_arm(&world->robot.right_arm);

        m4_translate(&world->robot.left_arm.transform_to_arm, &world->robot.transform, 0, SHOULDER_OFFSET_Y, SHOULDER_OFFSET_Z);
        compute_arm(&world->robot.left_arm);

#if 0
        m4_translate(&right_hip_offset, &world->robot.transform, 0, -HIP_OFFSET_Y, -HIP_OFFSET_Z);
        m4_rotate(&right_leg_transform, &right_hip_offset, world->robot.hip_yaw_pitch,
                  0.0, 0.70710678118655, 0.70710678118655);
        compute_leg(&right_leg_transform, &world->robot.right_leg);

        m4_translate(&left_hip_offset, &world->robot.transform, 0, HIP_OFFSET_Y, -HIP_OFFSET_Z);
        m4_rotate(&left_leg_transform, &left_hip_offset, world->robot.hip_yaw_pitch,
                  0.0, 0.70710678118655, -0.70710678118655);
        compute_leg(&left_leg_transform, &world->robot.left_leg);
#endif

        check_collisions(world); 

        m4_extract_translation(&r_pos, &world->robot.right_arm.transform_to_hand);
        m4_extract_translation(&l_pos, &world->robot.left_arm.transform_to_hand);
        m4_transform_i(&cup_bowl_center, &world->robot.right_arm.transform_to_hand,
                       0.0, 0.0, CUP_BOWL_HEIGHT + CUP_GRIP_HEIGHT/2.0, 1.0);

        double hand_dist = v2_dist(&l_pos.v2, &cup_bowl_center.v2);
        bool xy_align = hand_dist < (CUP_DIAMETER / 2.0 - BALL_RADIUS);

        bool left_above_right = cup_bowl_center.z < l_pos.z;

        bool hands_align = xy_align && left_above_right;

        v3_sub(&cup_up, &cup_bowl_center.v3, &r_pos.v3);
        v3_norm(&cup_up_norm, &cup_up);

        cup_is_up = cup_up_norm.z > RHAND_UP_GOAL;

        m4_transform_i(&left_hand_ball_down, &world->robot.left_arm.transform_to_hand, 0.0, -1.0, 0.0, 1.0);
        v3_sub(&ball_down, &left_hand_ball_down.v3, &l_pos.v3);
        v3_norm(&ball_down_norm, &ball_down);

        bool ball_is_down = ball_down_norm.z > LHAND_UP_GOAL;

        world->in_goal = hands_align && cup_is_up && ball_is_down;

        world->dist_to_goal = hand_dist
                + fmax(0.0, l_pos.z - cup_bowl_center.z)
                + (1 - cup_up_norm.z)
                + (1 - ball_down_norm.z);

        world->is_clear = cup_is_up && !world->in_collision;
}


static void
set_config(nao_world_t *world, const double *config)
{
        check_angle(R_SHOULDER_PITCH, config[0]);
        world->robot.right_arm.shoulder_pitch = config[0];

        check_angle(R_SHOULDER_ROLL, config[1]);
        world->robot.right_arm.shoulder_roll = config[1];

        check_angle(R_ELBOW_YAW, config[2]);
        world->robot.right_arm.elbow_yaw = config[2];

        check_angle(R_ELBOW_ROLL, config[3]);
        world->robot.right_arm.elbow_roll = config[3];

        check_angle(R_WRIST_YAW, config[4]);
        world->robot.right_arm.wrist_yaw = config[4];

        check_angle(L_SHOULDER_PITCH, config[5]);
        world->robot.left_arm.shoulder_pitch = config[5];

        check_angle(L_SHOULDER_ROLL, config[6]);
        world->robot.left_arm.shoulder_roll = config[6];

        check_angle(L_ELBOW_YAW, config[7]);
        world->robot.left_arm.elbow_yaw = config[7];

        check_angle(L_ELBOW_ROLL, config[8]);
        world->robot.left_arm.elbow_roll = config[8];

        check_angle(L_WRIST_YAW, config[9]);
        world->robot.left_arm.wrist_yaw = config[9];

        /* set_right_shoulder_pitch(nao, config[0]); */
        /* set_right_shoulder_roll(nao, config[1]); */
        /* set_right_elbow_yaw(nao, config[2]); */
        /* set_right_elbow_roll(nao, config[3]); */
        /* set_right_wrist_yaw(nao, config[4]); */

        /* set_left_shoulder_pitch(nao, config[5]); */
        /* set_left_shoulder_roll(nao, config[6]); */
        /* set_left_elbow_yaw(nao, config[7]); */
        /* set_left_elbow_roll(nao, config[8]); */
        /* set_left_wrist_yaw(nao, config[9]); */

        compute(world);
}


static double
nao_dist(const double *a, const double *b)
{
        double sum = 0;
        double d;
        int i;

        for (i=0 ; i<DIMENSIONS ; ++i) {
                d = b[i] - a[i];
                sum += d*d;
        }

        return sqrt(sum);
}

static bool
nao_in_goal(void *system_arg, const double *config)
{
        nao_world_t *world = (nao_world_t*)system_arg;

        set_config(world, config);
        
        return world->in_goal;
}

static bool
nao_clear(void *system_arg, const double *config)
{
        nao_world_t *world = (nao_world_t*)system_arg;

        set_config(world, config);

        return world->is_clear;

        /* return validate_cup_is_up(world) &&
           !validate_is_self_collision(&world->nao); */
}

static bool
nao_link_impl(nao_world_t *world, const double *a, const double *b)
{
        double d = nao_dist(a, b);
        double m[DIMENSIONS];
        int i;

        if (d < DISCRETIZATION) {
                return true;
        }

        for (i=0 ; i<DIMENSIONS ; ++i) {
                m[i] = (a[i] + b[i]) / 2.0;
        }

        return nao_clear(world, m)
                && nao_link_impl(world, a, m)
                && nao_link_impl(world, m, b);
}

static bool
nao_link(void *system_arg, const double *a, const double *b)
{
        nao_world_t *world = (nao_world_t*)system_arg;

        /*
         * clear(a) and clear(b) are assumed since the calling PRRT
         * calls clear on all samples before attempting to link
         * them.
         */

        return nao_link_impl(world, a, b);
}

static void *
nao_system_data_alloc(int thread_no, const double *sample_min, const double *sample_max)
{
        return struct_alloc(nao_world_t);
}

static void
nao_system_data_free(void *system)
{
        free(system);
}

prrts_system_t *
naocup_create_system()
{
        prrts_system_t *system;

        if ((system = struct_alloc(prrts_system_t)) == NULL)
                return NULL;

        memset(system, 0, sizeof(system));

        system->dimensions = DIMENSIONS;

        system->init = nao_init_config;
        system->min = nao_min_config;
        system->max = nao_max_config;
        system->target = nao_target_config;

        system->system_data_alloc_func = nao_system_data_alloc;
        system->system_data_free_func = nao_system_data_free;
        system->dist_func = nao_dist;
        system->in_goal_func = nao_in_goal;
        system->clear_func = nao_clear;
        system->link_func = nao_link;

        return system;
}

void
naocup_free_system(prrts_system_t *system)
{
        free(system);
}

static void
usage(char *prog_name)
{
        printf("usage: %s -t thread_count -n sample_count\n", prog_name);
        exit(1);
}

int
naocup_main(int argc, char *argv[])
{
        prrts_system_t *system;
        prrts_options_t options;
        prrts_solution_t *solution;
        long thread_count = 16;
        int i, j, ch;
        long sample_count = 2500;
        double sum;
        hrtimer_t start_time;
        hrtimer_t duration;
        char *ep;

        while ((ch = getopt(argc, argv, "t:n:")) != -1) {
                switch (ch) {
                case 't':
                        thread_count = strtol(optarg, &ep, 10);
                        if (thread_count < 1 || *ep != '\0') {
                                warnx("invalid thread count -t argument -- %s", optarg);
                                usage(argv[0]);
                        }
                        break;                                
                case 'n':
                        sample_count = strtol(optarg, &ep, 10);
                        if (sample_count < 1 || *ep != '\0') {
                                warnx("invalid sample count -n argument -- %s", optarg);
                                usage(argv[0]);
                        }
                        break;
                case '?':
                default:
                        usage(argv[0]);
                }
        }

        system = naocup_create_system();

        memset(&options, 0, sizeof(options));
        options.gamma = 5.0;
        options.regional_sampling = true;
        options.samples_per_step = 1;

        start_time = hrtimer_get();
        solution = prrts_run_for_samples(system, &options, thread_count, sample_count);
        duration = hrtimer_get() - start_time;

        naocup_free_system(system);

        printf("%d samples computed in %f seconds on %d threads (%f samples/second)\n",
               (int)sample_count, duration / (double)HRTICK, (int)thread_count,
               sample_count * (double)HRTICK / duration);

        if (solution == NULL) {
                printf("No solution found\n");
        } else {
                printf("Solution found, length=%u, cost=%f\n", (unsigned)solution->path_length, solution->path_cost);
                printf("angleList = [\\\n");
                for (i=0 ; i<DIMENSIONS ; ++i) {
                        printf("  [ ");
                        for (j=1 ; j<solution->path_length ; ++j) {
                                printf("%f, ", solution->configs[j][i]);
                        }
                        printf("], \\\n");
                }
                printf("]\n");
                printf("times = [\\\n");
                for (i=0 ; i<DIMENSIONS ; ++i) {
                        sum = 0.0;
                        printf("  [ ");
                        for (j=1 ; j<solution->path_length ; ++j) {
                                sum += nao_dist(solution->configs[j-1], solution->configs[j]);
                                printf("%f, ", sum);
                        }
                        printf("], \\\n");
                }
                printf("]\n");
        }

        return 0;
}
