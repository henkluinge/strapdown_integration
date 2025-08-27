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

// clear && gcc -I ./include ./src/process_strapdown_integration.c ./src/vectors.c ./src/strapdown_integration.c ./src/helper.c -lm -o strapdown && ./strapdown
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vectors.h"
#include "strapdown_integration.h"
#include "helper.h"

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

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

    quat q_ls, delta_q, q_ref;
    quat_identity(&q_ls); // Initial orientation
    double y_gyr[3], y_acc[3];
    double v_ref[3]; 
    double v_sdi[3] = {0.0, 0.0, 0.0}; // Initial velocity
    double t = 0.0,  t_prev = 0.0, sampleTime = 1; // Time variable
    
    printf("Reading log file\n");

    // Reference csv.
    FILE* reference_file = fopen(csv_file_path, "r");
    if (reference_file == NULL) {
        perror("Error opening file");
        return EXIT_FAILURE;
    }
   
    // Output log
    FILE* output_file = OpenOutputFile(csv_file_path);
    // Write header
    fprintf(output_file, "t,q_ref_w,q_ref_x,q_ref_y,q_ref_z, v_ref_x, v_ref_y,v_ref_z, q_ls_w,q_ls_x,q_ls_y,q_ls_z, v_sdi_x, v_sdi_x, v_sdi_x\n");

    // Read Header
    char line[512];
    printf("Header %s", fgets(line, sizeof(line), reference_file));

    // Starting orientation. 
    // By convention, the first sample (t=i=0) is used to set the initial orientation.
    fgets(line, sizeof(line), reference_file);
    WriteSampleToLog(output_file, t, q_ls, v_ref, q_ls, v_sdi);

    while (fgets(line, sizeof(line), reference_file)) {

        ParseRefCsvLine(line, &t, y_gyr, y_acc, &q_ref, v_ref);
        sampleTime = t - t_prev; // Sample time could theoretically change per samlpe. E.g. wireless IMU.
        t_prev = t;

        StrapdownIntegrationOnestep(y_gyr, y_acc, sampleTime, &q_ls, v_sdi);

        // // Print current state to console
        PrintVector(v_ref);

        // printf("%.4f  ", sampleTime);
        // printf("%.4f  ", t);
        // PrintVector(y_gyr);

        // quat_fprint(stdout, &q_ref);
        // quat_fprint(stdout, &q_ls);
        // printf("\n");

        WriteSampleToLog(output_file, t, q_ref, v_ref, q_ls, v_sdi);

    }
    fclose(reference_file);
    fclose(output_file);
    
    printf("Finished reading log file\n");
    printf("Use compare_c_implementation_with_reference.ipynb to make a graphical comparison.\n");
}
