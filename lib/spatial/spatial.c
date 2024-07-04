//
// Created by liyutong on 2024/5/10.
//
#include <math.h>

#include "spatial.h"

/**
 * @brief Convert quaternion to euler angles
 * @param q
 * @param[out] e
 * @param q
 * @param e
**/
void spatial_quaternion_to_euler(const Quaternion *q, Euler *e) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1 - 2 * (q->x * q->x + q->y * q->y);
    e->roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1) {
        e->pitch = copysignf(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        e->pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
    e->yaw = atan2f(siny_cosp, cosy_cosp);
}


/**
 * @brief Convert quaternion to euler angles in degrees
 * @param q
 * @param[out] e_deg
**/
void spatial_quaternion_to_euler_deg(const Quaternion *q, Euler *e_deg) {
    Euler e_rad;
    spatial_quaternion_to_euler(q, &e_rad);

    // Constants for conversion
    const float RAD_TO_DEG = 180.0f / M_PI;

    // Convert each angle from radians to degrees
    e_deg->roll = e_rad.roll * RAD_TO_DEG;
    e_deg->pitch = e_rad.pitch * RAD_TO_DEG;
    e_deg->yaw = e_rad.yaw * RAD_TO_DEG;
}

/**
 * @brief Multiply a vector by a scalar and add another vector
 * @param v1
 * @param v2
 * @param coef
 * @param[out] v
**/
void spatial_vector_multiply_plus(const Vector3 *v1, const Vector3 *v2, float coef, Vector3 *v) {
    v->x = v1->x + coef * v2->x;
    v->y = v1->y + coef * v2->y;
    v->z = v1->z + coef * v2->z;

}

float spatial_vector_norm(const Vector3 *v) {
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

/**
 * @brief Convert quaternion to rotation matrix
 * @param q
 * @param[out] R
**/
void spatial_quaternion_to_rotation_matrix(const Quaternion *q, Matrix3x3 *R) {
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;
    R->m[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    R->m[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    R->m[0][2] = 2.0f * (q0 * q2 + q1 * q3);
    R->m[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    R->m[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    R->m[1][2] = 2.0f * (q2 * q3 - q0 * q1);
    R->m[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    R->m[2][1] = 2.0f * (q0 * q1 + q2 * q3);
    R->m[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
}

void spatial_rotation_matrix_to_quaternion(const Matrix3x3 *R, Quaternion *q) {

}

/**
 * @brief Multiply two quaternions
 * @param q1
 * @param q2
 * @param[out] q
**/
void spatial_quaternion_multiply(const Quaternion *q1, const Quaternion *q2, Quaternion *q){
    q->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    q->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    q->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    q->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

/**
 * @brief Convert quaternion to axis-angle representation
 * @param q
 * @param[out] axis
 * @param[out] angle
**/
void spatial_quaternion_to_axis_angle(const Quaternion *q, Vector3 *axis, float *angle) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    Quaternion q_norm = {q->w / norm , q->x / norm, q->y / norm, q->z / norm};
    *angle = 2.0f * acosf(q_norm.w);
    float sin_theta_over_2 = sqrtf(1.0f - q_norm.w * q_norm.w);
    if (fabsf(sin_theta_over_2) < 1e-6) {
        // If the quaternion is close to identity, the axis is arbitrary
        axis->x = 1.0f;
        axis->y = 0.0f;
        axis->z = 0.0f;
        return;
    } else {
        axis->x = q_norm.x / sin_theta_over_2;
        axis->y = q_norm.y / sin_theta_over_2;
        axis->z = q_norm.z / sin_theta_over_2;
    }
}


/**
 * @brief Compute angle difference(rad) between two angles(in quaternion)
 * @param q1
 * @param q2
 * @param[out] axis
 * @param[out] angle
 * @return
 */
void spatial_rotation_diff_quaternions(const Quaternion * q1, const Quaternion *q2, Vector3 *axis, float *angle){
    Quaternion q2_inverse = {q2->w, -q2->x, -q2->y, -q2->z};
    Quaternion q;
    spatial_quaternion_multiply(q1, &q2_inverse, &q);
    spatial_quaternion_to_axis_angle(&q, axis, angle);
}