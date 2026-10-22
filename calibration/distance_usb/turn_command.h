#ifndef CALIBRATION_TURN_COMMAND_H
#define CALIBRATION_TURN_COMMAND_H
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
/* turn_left 45 / turn_right 90, also accept turn_left_90 compatibility form. */
static inline bool parse_turn_command(const char *line, double *degrees) {
    const char *number;
    double direction;
    if (strncmp(line,"turn_left",9U)==0 && (line[9]==' ' || line[9]=='_')) {
        number=line+10; direction=1.0;
    } else if (strncmp(line,"turn_right",10U)==0 && (line[10]==' ' || line[10]=='_')) {
        number=line+11; direction=-1.0;
    } else { return false; }
    char *end;
    double value=strtod(number,&end);
    if (end==number || !isfinite(value) || value<15.0 || value>180.0) { return false; }
    while (*end==' ') { ++end; }
    if (*end!='\0') { return false; }
    *degrees=direction*value;
    return true;
}
#endif
