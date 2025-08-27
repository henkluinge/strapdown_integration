// This function works on three types:
// - quat, the well known quaterion.
// - Array of doubles.

#ifndef VECTORS_H
#define VECTORS_H

#include <math.h>
#include <stdio.h>
#include <string.h>  // for memcpy

#ifdef __cplusplus
extern "C" {
#endif

// Type definition

// Quaternion structure
typedef struct {
    double w, x, y, z;
} quat;


// ========== ARRAY OPERATIONS ==========
void add_arrays(double a[3], double b[3], double result[3]);

void add_to_array(double a[3], const double b[3]);

void scalar_multiply_array_in_place(double a[3], double T);

// ========== QUATERNION OPERATIONS ==========

// Create a quaternion
void quat_create(double w, double x, double y, double z, quat* q);

// Identity quaternion
void quat_identity(quat* q);

// Siminlar to 'quat_from_axis_angle', but the length of the vector is the angle in radians.
void quat_from_rotation(double* y_gyr, quat* q);

// Create quaternion from axis-angle (angle in radians)
void quat_from_axis_angle(  const double* axis, double angle, quat* q);

// Create quaternion from Euler angles (roll, pitch, yaw in radians)
void quat_from_euler(double roll, double pitch, double yaw, quat* q);

// Quaternion multiplication
void quat_multiply(quat* a, quat* b, quat* result);

// Quaternion magnitude
double quat_length(  quat* q);

// Normalize quaternion
void quat_normalize(  quat* q, quat* result);

// Quaternion conjugate
void quat_conjugate(  quat* q, quat* result);

// ========== QUATERNION OPERATIONS ==========

void quat_rotate_vector_copy(const quat* q_ls, const double v_in[3], double v_out[3]);

// ========== UTILITY FUNCTIONS ==========

// Convert degrees to radians
double deg_to_rad(double degrees);

// Convert radians to degrees
double rad_to_deg(double radians);

// Print quaternion to stdout
void quat_print(  quat* q);

// Print quaternion to file
void quat_fprint(FILE* file, quat* q);

#ifdef __cplusplus
}
#endif

#endif // VECMATH_H