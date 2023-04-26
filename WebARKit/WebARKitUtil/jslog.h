#ifndef JSLOG_H
#define JSLOG_H

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

enum JSLogLevel {
  JS_LOG_LEVEL_DEBUG = 0,
  JS_LOG_LEVEL_INFO,
  JS_LOG_LEVEL_WARN,
  JS_LOG_LEVEL_ERROR,
  JS_LOG_LEVEL_REL_INFO
};

#define JS_LOG_LEVEL_DEFAULT JS_LOG_LEVEL_DEBUG
inline int jsLogLevel = JS_LOG_LEVEL_DEFAULT;

#ifdef DEBUG_EM
#define JSLOGd(...) jsLog(NULL, JS_LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define JSLOGd(...)
#endif
#define JSLOGi(...) jsLog(NULL, JS_LOG_LEVEL_INFO, __VA_ARGS__)
#define JSLOGw(...) jsLog(NULL, JS_LOG_LEVEL_WARN, __VA_ARGS__)
#define JSLOGe(...) jsLog(NULL, JS_LOG_LEVEL_ERROR, __VA_ARGS__)
#define JSLOGperror(s)                                                         \
  jsLog(NULL, JS_LOG_LEVEL_ERROR, ((s != NULL) ? "%s: %s\n" : "%s%s\n"),       \
        ((s != NULL) ? s : ""), strerror(errno))

inline void jsLogv(const char *tag, const int logLevel, const char *format, va_list ap) {
  va_list ap2;
  size_t len;
  const char *logLevelStrings[] = {"debug", "info", "warning", "error"};
  const size_t logLevelStringsCount =
      (sizeof(logLevelStrings) / sizeof(logLevelStrings[0]));
  size_t logLevelStringLen;

  if (logLevel < jsLogLevel)
    return;
  if (!format || !format[0])
    return;

  // Count length required to unpack varargs.
  va_copy(ap2, ap);

  len = std::vsnprintf(NULL, 0, format, ap2);

  va_end(ap2);
  if (len < 1)
    return;

  // Add characters required for logLevelString.
  if (logLevel >= 0 && logLevel < (int)logLevelStringsCount) {
    logLevelStringLen =
        9 + strlen(logLevelStrings[logLevel]); // +3 for brackets and a space,
                                               // e.g. "[debug] ".
  } else {
    logLevelStringLen = 0;
  }

  std::vector<char> buf(1+std::vsnprintf(nullptr, 0, format, ap));

  if (logLevelStringLen > 0) {
    std::snprintf(buf.data(), logLevelStringLen + 1, "JSLOG [%s] ",
                  logLevelStrings[logLevel]);
  }

  std::vsnprintf(buf.data() + logLevelStringLen, len + 1, format, ap);
  va_end(ap);
  len += logLevelStringLen;

  std::fprintf(stderr, "%s\n", buf.data());
}

inline void jsLog(const char *tag, const int logLevel, const char *format,
                  ...) {
  if (logLevel < jsLogLevel)
    return;
  if (!format || !format[0])
    return;

  va_list ap;
  va_start(ap, format);
  jsLogv(tag, logLevel, format, ap);
  va_end(ap);
}
#endif