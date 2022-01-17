#ifndef WEBARKIT_LOG_H
#define WEBARKIT_LOG_H

#include <emscripten.h>

void webarkitLOGi(char* message);

void webarkitLOGi(char* message, double * format);

void webarkitLOGi(char* message, float * format);

void webarkitLOGi(char* message, char * format);

void webarkitLOGi(char* message, const char * format);

void webarkitLOGi(char* message, int  format);

void webarkitLOGe(char* message);

void webarkitLOGe(char* message, double * format);

void webarkitLOGe(char* message, float * format);

void webarkitLOGe(char* message, char * format);

void webarkitLOGe(char* message, const char * format);

void webarkitLOGe(char* message, int  format);

void webarkitLOGw(char* message);

void webarkitLOGw(char* message, double * format);

void webarkitLOGw(char* message, float * format);

void webarkitLOGw(char* message, char * format);

void webarkitLOGw(char* message, const char * format);

void webarkitLOGw(char* message, int  format);


#endif // #ifndef WEBARKIT_LOG_H