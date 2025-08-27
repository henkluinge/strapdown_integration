#include "vectors.h"
#include <stdio.h>


// ========== ARRAY OPERATIONS ==========
// Function to add two 3-element double arrays element-wise
void add_arrays(double a[3], double b[3], double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = a[i] + b[i];
    }
}

void scalar_multiply_array_in_place(double* a, double T) {
    for (int i = 0; i < 3; i++) {
        a[i] = a[i] * T;
    }
}

void add_to_array(double a[3], const double b[3]) {
    for (int i = 0; i < 3; i++) {
        a[i] += b[i];
    }
}

void normalize3(const double *v, double *result) {
    double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (norm == 0.0) {
        result[0] = result[1] = result[2] = 0.0;
        return;
    }
    result[0] = v[0] / norm;
    result[1] = v[1] / norm;
    result[2] = v[2] / norm;
}
// ========== QUATERNION OPERATIONS ==========


// Create a quaternion
void quat_create(double w, double x, double y, double z, quat* q) {
    q->w = w;
    q->x = x;
    q->y = y;
    q->z = z;
}

// Identity quaternion
void quat_identity(quat* q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

// Create quaternion from axis-angle (angle in radians)
void quat_from_rotation(double* y_gyr, quat* q) {
    double v_norm = sqrt(y_gyr[0]*y_gyr[0] + y_gyr[1]*y_gyr[1] + y_gyr[2]*y_gyr[2]);
    if (v_norm > 1e-6) {
        double v_normed[3] = {y_gyr[0]/v_norm, y_gyr[1]/v_norm, y_gyr[2]/v_norm};
        double v_norm_div = 0.5*v_norm;
        q->w = cos(v_norm_div);
        q->x = v_normed[0]*sin(v_norm_div);
        q->y = v_normed[1]*sin(v_norm_div);
        q->z = v_normed[2]*sin(v_norm_div);
    } else {
        // No rotation
        quat_identity(q);
    }
}

// Create quaternion from axis-angle (angle in radians)
void quat_from_axis_angle(  const double* axis, double angle, quat* q) {
    double axis_normalized[3];
    normalize3(axis, axis_normalized);
    
    double half_angle = angle * 0.5f;
    double sin_half = sinf(half_angle);
    
    q->w = cosf(half_angle);
    q->x = axis_normalized[0] * sin_half;
    q->y = axis_normalized[1] * sin_half;
    q->z = axis_normalized[2] * sin_half;
}

// Create quaternion from Euler angles (roll, pitch, yaw in radians)
void quat_from_euler(double roll, double pitch, double yaw, quat* q) {
    double cr = cosf(roll * 0.5f);
    double sr = sinf(roll * 0.5f);
    double cp = cosf(pitch * 0.5f);
    double sp = sinf(pitch * 0.5f);
    double cy = cosf(yaw * 0.5f);
    double sy = sinf(yaw * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

// Quaternion multiplication
void quat_multiply(quat* a, quat* b, quat* result) {
    result->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    result->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    result->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    result->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

// Quaternion magnitude
double quat_length(  quat* q) {
    return sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

// Normalize quaternion
void quat_normalize(  quat* q, quat* result) {
    double len = quat_length(q);
    if (len > 0.0f) {
        double inv_len = 1.0f / len;
        result->w = q->w * inv_len;
        result->x = q->x * inv_len;
        result->y = q->y * inv_len;
        result->z = q->z * inv_len;
    } else {
        quat_identity(result);
    }
}

// Quaternion conjugate
void quat_conjugate(  quat* q, quat* result) {
    result->w = q->w;
    result->x = -q->x;
    result->y = -q->y;
    result->z = -q->z;
}


// Alternative version that returns a new vector instead of modifying in-place
void quat_rotate_vector_copy(const quat* q_ls, const double v_in[3], double v_out[3]) {
    // Extract quaternion components
    double qw = q_ls->w;
    double qx = q_ls->x;
    double qy = q_ls->y;
    double qz = q_ls->z;
    
    // Extract vector components
    double vx = v_in[0];
    double vy = v_in[1];
    double vz = v_in[2];
    
    // Apply quaternion rotation formula
    // First cross product: cross(q.xyz, v)
    double cx = qy * vz - qz * vy;
    double cy = qz * vx - qx * vz;
    double cz = qx * vy - qy * vx;
    
    // Add q.w * v to the cross product
    cx += qw * vx;
    cy += qw * vy;
    cz += qw * vz;
    
    // Second cross product: cross(q.xyz, (cross(q.xyz, v) + q.w * v))
    double ccx = qy * cz - qz * cy;
    double ccy = qz * cx - qx * cz;
    double ccz = qx * cy - qy * cx;
    
    // Final result: v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
    v_out[0] = vx + 2.0 * ccx;
    v_out[1] = vy + 2.0 * ccy;
    v_out[2] = vz + 2.0 * ccz;
}

// ========== UTILITY FUNCTIONS ==========

// Convert degrees to radians
double deg_to_rad(double degrees) {
    return degrees * M_PI / 180.0f;
}

// Convert radians to degrees
double rad_to_deg(double radians) {
    return radians * 180.0f / M_PI;
}

// Print quaternion to stdout
void quat_print(  quat* q) {
    printf("(%.3f, %.3f, %.3f, %.3f)\n", q->w, q->x, q->y, q->z);
}

// Print quaternion to file
void quat_fprint(FILE* file, quat* q) {
    fprintf(file, "(%.3f, %.3f, %.3f, %.3f)", q->w, q->x, q->y, q->z);
}