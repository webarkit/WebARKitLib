#ifndef WEBARKIT_LOG_H
#define WEBARKIT_LOG_H

#include <emscripten.h>

void webarkitLOGi(char* message);

void webarkitLOGi(char* message, double * format);

void webarkitLOGi(char* message, float * format);

void webarkitLOGi(char* message, char * format);

void webarkitLOGi(char* message, const char * format);

void webarkitLOGi(char* message, int  format);


#endif // #ifndef WEBARKIT_LOG_H