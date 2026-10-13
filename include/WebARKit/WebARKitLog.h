// The Emscripten logger. Its guard differs from the native logger's
// (WebARKit/include/WebARKitLog.h, WEBARKIT_LOG_H), so a file can include both (#84).
#ifndef WEBARKIT_LOG_EM_H
#define WEBARKIT_LOG_EM_H

// Only lib/SRC/WebARKit/WebARKitLog.cpp needs it (EM_ASM); files that include this header
// under Emscripten still get it.
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
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

void webarkitLOGd(const std::string &message);

#endif // #ifndef WEBARKIT_LOG_EM_H