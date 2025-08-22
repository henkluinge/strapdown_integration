// Copyright (C) 2024 Henk Luinge henk.luinge@gmail.com
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

// gcc -I ./include ./src/process_strapdown_integration.c ./src/Quaternion.c ./src/strapdown_integration.c -lm -o strapdown && ./strapdown
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Quaternion.h"
#include "strapdown_integration.h"

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

void WriteSampleToLog(FILE* output_file, Quaternion q_ref, Quaternion q_ls, float t){
        // Write the current state to the output file
        fprintf(output_file, "%.4f,", t);
        fprintf(output_file, "%.6f,%.6f,%.6f,%.6f,", q_ref.w, q_ref.v[0], q_ref.v[1], q_ref.v[2]);
        fprintf(output_file, "%.6f,%.6f,%.6f,%.6f", q_ls.w, q_ls.v[0], q_ls.v[1], q_ls.v[2]);
        fprintf(output_file,"\n");
    }

int main(int argc, char *argv[])
{

    // Check if the correct number of arguments is provided
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <path_to_csv_file>\n", argv[0]);
        return 1;
    }
    
    // argv[1] contains the CSV file path
    char *csv_file_path = argv[1];
    
    printf("CSV file path: %s\n", csv_file_path);

    // Output file path
    char output_file_path[512];
    char *dot = strrchr(csv_file_path, '.');
    if (dot) {
        size_t prefix_len = dot - csv_file_path;
        strncpy(output_file_path, csv_file_path, prefix_len);
        output_file_path[prefix_len] = '\0';
        strcat(output_file_path, "_with_c");
        strcat(output_file_path, dot);
        printf("Output file path: %s\n", output_file_path);
        
    } else {
        snprintf(output_file_path, sizeof(output_file_path), "%s_with_c.csv", csv_file_path);
    }


    Quaternion q_ls, delta_q, q_ref;
    Quaternion_setIdentity(&q_ls);   // The identity quaternion represents no rotation
    double y_gyr[3], y_acc[3]; 
    float t = 0.0,  t_prev = 0.0, sampleTime = 1; // Time variable
    
    printf("Reading log file\n");

    // Reference csv.
    FILE* reference_file = fopen(csv_file_path, "r");
    if (reference_file == NULL) {
        perror("Error opening file");
        return EXIT_FAILURE;
    }
    // Read Header
    char line[256];
    printf("%s", fgets(line, sizeof(line), reference_file));
    
    // Output log
    FILE* output_file = fopen(output_file_path, "w");
    if (output_file == NULL) {
        perror("Error opening output file");
        return EXIT_FAILURE;
    }
    // Write header
    fprintf(output_file, "t,q_ref_w,q_ref_x,q_ref_y,q_ref_z,q_ls_w,q_ls_x,q_ls_y,q_ls_z\n");

    // Starting orientation. By convention, the first sample (t=i=0) is used to set the initial orientation.
    fgets(line, sizeof(line), reference_file);
    ParseRefCsvLine(line, &t_prev, y_gyr, y_acc, &q_ls);
    WriteSampleToLog(output_file, q_ls, q_ls, t);
    printf("Starting orientation\n");
    Quaternion_fprint(stdout, &q_ls);
    printf("\n");

    while (fgets(line, sizeof(line), reference_file)) {
            
        ParseRefCsvLine(line, &t, y_gyr, y_acc, &q_ref);
        sampleTime = t - t_prev; // Sample time could theoretically change per samlpe. E.g. wireless IMU.
        t_prev = t;

        StrapdownIntegrationOnestep(y_gyr, sampleTime, &q_ls);

        // Print the current state
        printf("%.4f  ", sampleTime);
        printf("%.4f  ", t);
        PrintVector(y_gyr);

        Quaternion_fprint(stdout, &q_ref);
        Quaternion_fprint(stdout, &q_ls);
        printf("\n");

        WriteSampleToLog(output_file, q_ref, q_ls, t);
    }
    fclose(reference_file);
    fclose(output_file);
    
    printf("Finished reading log file\n");
    printf("Use compare_c_implementation_with_reference.ipynb to make a graphical comparison.\n");
}
