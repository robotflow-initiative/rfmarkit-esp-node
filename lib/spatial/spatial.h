//
// Created by liyutong on 2024/5/10.
//

#ifndef SPATIAL_H
#define SPATIAL_H

typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    float x, y, z;
} Vector3;

typedef struct {
    float roll, pitch, yaw;
} Euler;

typedef struct {
    float m[3][3];
} Matrix3x3;

void spatial_quaternion_to_euler(const Quaternion *q, Euler *e);

void spatial_quaternion_to_euler_deg(const Quaternion *q, Euler *e_deg);

void spatial_vector_multiply_plus(const Vector3 *v1, const Vector3 *v2, float coef, Vector3 *v);

float spatial_vector_norm(const Vector3 *v);

void spatial_quaternion_to_rotation_matrix(const Quaternion *q, Matrix3x3 *R);

void spatial_rotation_matrix_to_quaternion(const Matrix3x3 *R, Quaternion *q);

void spatial_quaternion_multiply(const Quaternion *q1, const Quaternion *q2, Quaternion *q);

void spatial_quaternion_to_axis_angle(const Quaternion *q, Vector3 *axis, float *angle);

void spatial_rotation_diff_quaternions(const Quaternion *q1, const Quaternion *q2, Vector3 *axis, float *angle);

#endif //SPATIAL_H
