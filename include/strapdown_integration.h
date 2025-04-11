#pragma once
#include <stdlib.h>
#include <string.h>
#include "Quaternion.h"

void PrintVector(double* vector);

void StrapdownIntegrationOnestep(double* y_gyr, double sampleTime, Quaternion* orientation);

void ParseRefCsvLine(char* line, float* time,  double* y_gyr, double* y_acc, Quaternion* q_ref);