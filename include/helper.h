#ifndef HELPER_H
#define HELPER_H

#include <stdio.h>
#include "Quaternion.h"

void WriteSampleToLog(FILE* output_file, Quaternion q_ref, Quaternion q_ls, float t);

#endif // HELPER_H