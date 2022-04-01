#ifndef WEBARKIT_LOG_H
#define WEBARKIT_LOG_H

#include <emscripten.h>
#include <iostream>

void webarkitLOGi(const std::string &message);

void webarkitLOGi(const std::string &message, double * format);

void webarkitLOGi(const std::string &message, float * format);

void webarkitLOGi(const std::string &message, char * format);

void webarkitLOGi(const std::string &message, const char * format);

void webarkitLOGi(const std::string &message, int  format);

void webarkitLOGe(const std::string &message);

void webarkitLOGe(const std::string &message, double * format);

void webarkitLOGe(const std::string &message, float * format);

void webarkitLOGe(const std::string &message, char * format);

void webarkitLOGe(const std::string &message, const char * format);

void webarkitLOGe(const std::string &message, int  format);

void webarkitLOGw(const std::string &message);

void webarkitLOGw(const std::string &message, double * format);

void webarkitLOGw(const std::string &message, float * format);

void webarkitLOGw(const std::string &message, char * format);

void webarkitLOGw(const std::string &message, const char * format);

void webarkitLOGw(const std::string &message, int  format);


#endif // #ifndef WEBARKIT_LOG_H