#include "helper.h"

void WriteSampleToLog(FILE* output_file, Quaternion q_ref, Quaternion q_ls, float t){
    // Write the current state to the output file
    fprintf(output_file, "%.4f,", t);
    fprintf(output_file, "%.6f,%.6f,%.6f,%.6f,", q_ref.w, q_ref.v[0], q_ref.v[1], q_ref.v[2]);
    fprintf(output_file, "%.6f,%.6f,%.6f,%.6f", q_ls.w, q_ls.v[0], q_ls.v[1], q_ls.v[2]);
    fprintf(output_file,"\n");
}