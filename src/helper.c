#include "helper.h"

void WriteSampleToLog(FILE* output_file, double t, quat q_ref, double* v_ref, quat q_ls, double* v_sdi) {
    // Write the current state to the output file
    fprintf(output_file, "%.4f,", t);

    fprintf(output_file, "%.6f,%.6f,%.6f,%.6f,", q_ref.w, q_ref.x, q_ref.y, q_ref.z);
    fprintf(output_file, "%.6f,%.6f,%.6f,", v_ref[0], v_ref[1], v_ref[2]  );

    fprintf(output_file, "%.6f,%.6f,%.6f,%.6f,", q_ls.w, q_ls.x, q_ls.y, q_ls.z);
    fprintf(output_file, "%.6f,%.6f,%.6f", v_sdi[0], v_sdi[1], v_sdi[2]  );
    fprintf(output_file,"\n");
}

void GenerateOutputFilePath(const char* csv_file_path, char* output_file_path, size_t bufsize) {
    const char *dot = strrchr(csv_file_path, '.');
    if (dot) {
        size_t prefix_len = dot - csv_file_path;
        if (prefix_len < bufsize) {
            strncpy(output_file_path, csv_file_path, prefix_len);
            output_file_path[prefix_len] = '\0';
            strncat(output_file_path, "_with_c", bufsize - strlen(output_file_path) - 1);
            strncat(output_file_path, dot, bufsize - strlen(output_file_path) - 1);
        } else {
            // Fallback if buffer too small
            snprintf(output_file_path, bufsize, "%s_with_c.csv", csv_file_path);
        }
    } else {
        snprintf(output_file_path, bufsize, "%s_with_c.csv", csv_file_path);
    }
}

FILE* OpenOutputFile(const char* csv_file_path) {
    // Output file path by adding _with_c before the .csv extension
    char output_file_path[512];
    GenerateOutputFilePath(csv_file_path, output_file_path, sizeof(output_file_path));
    printf("Output file path: %s\n", output_file_path);
    FILE* output_file = fopen(output_file_path, "w");
    if (output_file == NULL) {
        perror("Error opening output file");
        return NULL;
    }
    return output_file;
}