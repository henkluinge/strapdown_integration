// Various functions to process a strapdown integration log file.
// - Reading and writing logs.
// - Strapdown math.
// It is used for process_strapdown_integration.c.
#include <stdlib.h>
#include <string.h>
#include "vectors.h"

double GRAVITY_VECTOR[3] = {0.0, 0.0, -9.81};

void PrintVector(double* vector)
{
    printf("[%.3f, %.3f, %.3f]",
        vector[0], vector[1], vector[2]);
}

// Integrate the angular velocity to the next orientation, assuming the 
// angular velocity is constant over the sample time.
void StrapdownIntegrationOnestep(double* y_gyr, double* y_acc,  double sampleTime, quat* q_ls_pointer, double* v_sdi)
{

    printf("\n q_ls");
    quat_fprint(stdout, q_ls_pointer);
    printf("\n v_sdi ");
    PrintVector(v_sdi);
    printf("\n ");

    quat delta_q;
    quat_from_rotation(y_gyr, &delta_q);    
    // quat_normalize(&delta_q, &delta_q); // Ensure unit quaternion

    // Update orientation
    quat_multiply(q_ls_pointer, &delta_q, q_ls_pointer); 

    // # Accelerometer integration
    double y_acc_l[3];

    quat_rotate_vector_copy(q_ls_pointer, y_acc, y_acc_l); 
    scalar_multiply_array_in_place(y_acc_l, sampleTime); 
    add_to_array(v_sdi, y_acc_l);

    printf(" After SDI -> ");
    printf("\n q_ls ");
    quat_fprint(stdout, q_ls_pointer);
    printf("\n v_sdi ");
    PrintVector(v_sdi);
}

/* Extract data from one line of the reference log.
   Logfiles are generated using process_strapdown_integration.ipynb. They contain the reference values 
   for strapdown integration: time, gyroscope (3 ch), accelerometer (3ch) and reference quaternion (4ch).
*/
void ParseRefCsvLine(char* line, double* time, double* y_gyr,  double* y_acc, quat* q_ref, double* v_ref)
{
    // Time
    char* token = strtok(line, ",");
    if (token == NULL) {
        fprintf(stderr, "Error parsing time from line: %s\n", line);
        return;
    }
    *time = strtof(token, NULL); // Dereference the pointer to assign the value

    // Gyroscope data
    int i = 0;
    while (token != NULL && i < 3) {
        token = strtok(NULL, ",");
        y_gyr[i] = strtof(token, NULL);
        i++;
    }
    // Accelerometer data
    i = 0;
    while (token != NULL && i < 3) {
        token = strtok(NULL, ",");
        y_acc[i] = strtof(token, NULL);
        i++;
    }
    // Quaternion
    double w = strtof(strtok(NULL, ","), NULL);
    double v0 = strtof(strtok(NULL, ","), NULL);
    double v1 = strtof(strtok(NULL, ","), NULL);
    double v2 = strtof(strtok(NULL, ","), NULL);
    quat_create(w, v0, v1, v2, q_ref);

    // v_ref
    i = 0;
    while (token != NULL && i < 3) {
        token = strtok(NULL, ",");
        v_ref[i] = strtof(token, NULL);
        i++;
    }
}