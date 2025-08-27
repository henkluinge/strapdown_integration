#ifndef HELPER_H
#define HELPER_H

#include <stdio.h>
#include <string.h>
#include "vectors.h"

// void WriteSampleToLog(FILE* output_file, quat q_ref, quat q_ls, double t);
void WriteSampleToLog(FILE* output_file, double t, quat q_ref, double* v_ref, quat q_ls, double* v_sdi);

void GenerateOutputFilePath(const char* csv_file_path, char* output_file_path, size_t bufsize);

FILE* OpenOutputFile(const char* csv_file_path);

#endif // HELPER_H