#ifndef WEBARKITLOG_H
#define WEBARKITLOG_H

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

enum WEBARKITLogLevel {
  WEBARKIT_LOG_LEVEL_DEBUG = 0,
  WEBARKIT_LOG_LEVEL_INFO,
  WEBARKIT_LOG_LEVEL_WARN,
  WEBARKIT_LOG_LEVEL_ERROR,
  WEBARKIT_LOG_LEVEL_REL_INFO
};

#define WEBARKIT_LOG_LEVEL_DEFAULT WEBARKIT_LOG_LEVEL_DEBUG
inline int webarkitLogLevel = WEBARKIT_LOG_LEVEL_DEFAULT;

#ifdef DEBUG_EM
#define WEBARKITLOGd(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define WEBARKITLOGd(...)
#endif
#define WEBARKITLOGi(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_INFO, __VA_ARGS__)
#define WEBARKITLOGw(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_WARN, __VA_ARGS__)
#define WEBARKITLOGe(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_ERROR, __VA_ARGS__)
#define WEBARKITLOGperror(s)                                                         \
  webarkitLog(NULL, WEBARKIT_LOG_LEVEL_ERROR, ((s != NULL) ? "%s: %s\n" : "%s%s\n"),       \
        ((s != NULL) ? s : ""), strerror(errno))

inline void webarkitLogv(const char *tag, const int logLevel, const char *format, va_list ap) {
  va_list ap2;
  size_t len;
  const char *logLevelStrings[] = {"\033[37;40mdebug\033[0m", "\033[37;42minfo\033[0m", "\033[37;44mwarning\033[0m", "\033[37;41merror\033[0m"};
  const size_t logLevelStringsCount =
      (sizeof(logLevelStrings) / sizeof(logLevelStrings[0]));
  size_t logLevelStringLen;

  if (logLevel < webarkitLogLevel)
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
        13 + strlen(logLevelStrings[logLevel]); // +13 for WEBARKITLOG text and a space.
  } else {
    logLevelStringLen = 0;
  }

  std::vector<char> buf(1+std::vsnprintf(nullptr, 0, format, ap));

  if (logLevelStringLen > 0) {
    std::snprintf(buf.data(), logLevelStringLen + 1, "WEBARKITLOG %s ",
                  logLevelStrings[logLevel]);
  }

  std::vsnprintf(buf.data() + logLevelStringLen, len + 1, format, ap);
  //va_end(ap);
  len += logLevelStringLen;

  std::fprintf(stderr, "%s\n", buf.data());
}

inline void webarkitLog(const char *tag, const int logLevel, const char *format,
                  ...) {
  if (logLevel < webarkitLogLevel)
    return;
  if (!format || !format[0])
    return;

  va_list ap;
  va_start(ap, format);
  webarkitLogv(tag, logLevel, format, ap);
  va_end(ap);
}

#endif