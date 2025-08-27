#pragma once
#include <stdlib.h>
#include <string.h>

void PrintVector(double* vector);

void StrapdownIntegrationOnestep(double* y_gyr,double* y_acc, double sampleTime, quat* orientation, double* velocity);

void ParseRefCsvLine(char* line, double* time,  double* y_gyr, double* y_acc, quat* q_ref, double* v_ref);