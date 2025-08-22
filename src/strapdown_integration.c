// Various functions to process a strapdown integration log file.
// - Reading and writing logs.
// - Strapdown math.
// It is used for process_strapdown_integration.c.
#include <stdlib.h>
#include <string.h>
#include "Quaternion.h"

void PrintVector(double* vector)
{
    printf("[%.3f, %.3f, %.3f]",
        vector[0], vector[1], vector[2]);
}

// Integrate the angular velocity to the next orientation, assuming the 
// angular velocity is constant over the sample time.
void StrapdownIntegrationOnestep(double* y_gyr, double sampleTime, Quaternion* orientation)
{
    Quaternion delta_q;
    // Sloppy but fast orientation change.
    Quaternion_set(1, 0.5*sampleTime*y_gyr[0], 0.5*sampleTime*y_gyr[1], 0.5*sampleTime*y_gyr[2], &delta_q);
    Quaternion_multiply(orientation, &delta_q, orientation); // Update orientation
}

/* Extract data from one line of the reference log.
   Logfiles are generated using process_strapdown_integration.ipynb. They contain the reference values 
   for strapdown integration: time, gyroscope (3 ch), accelerometer (3ch) and reference quaternion (4ch).
*/
void ParseRefCsvLine(char* line, float* time, double* y_gyr,  double* y_acc, Quaternion* q_ref)
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
    Quaternion_set(w, v0, v1, v2, q_ref);
}