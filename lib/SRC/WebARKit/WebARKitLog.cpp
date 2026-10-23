#include <WebARKit/WebARKitLog.h>
// The native logger owns webarkitLogLevel and the WEBARKIT_LOG_LEVEL_* constants. Its include
// guard differs from this logger's (#84), so both headers can be included. webarkitLogLevel is
// declared extern "C" there, matching its definition in WebARKit/WebARKitLog.cpp.
#include <WebARKitLog.h>

const char * WARKTinfo = "%cℹ️[webarkit-info:]";
const char * WARKTinfoStyle = "color: #664400; background-color: #ffff99; border-radius: 4px; padding: 2px";
const char * WARKTerror = "%c🚩[webarkit-error:]";
const char * WARKTerrorStyle = "color: #ffffff; background-color: #ff0101; border-radius: 4px; padding: 2px";
const char * WARKTwarn = "%c⚠️[webarkit-warn:]";
const char * WARKTwarnStyle = "color: #774400; background-color: #ffff99; border-radius: 4px; padding: 2px";
const char * WARKTdebug = "%c🐞[webarkit-debug:]";
const char * WARKTdebugStyle = "color: #000000; background-color: #ffcc00; border-radius: 4px; padding: 2px";

// jsartoolkitNFT#677: this logger used to print regardless of webarkitLogLevel, so the adapter's
// start-up lines could not be silenced. Same comparison as the native logger: a message is
// skipped when its level is below the current level.
static inline bool logSuppressed(int level) {
    return level < webarkitLogLevel;
}

void webarkitLOGi(const std::string &message) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_INFO)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(infoHead + message, style);
    },
    message.c_str(),
    WARKTinfo,
    WARKTinfoStyle
    );
}

void webarkitLOGi(const std::string &message, double * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_INFO)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message.c_str(),
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(const std::string &message, float * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_INFO)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message.c_str(),
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(const std::string &message, char * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_INFO)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message.c_str(),
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(const std::string &message, const char * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_INFO)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message.c_str(),
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(const std::string &message, int  format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_INFO)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(infoHead + message, style, $3);
    },
    message.c_str(),
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGe(const std::string &message) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_ERROR)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.error(errorHead + message, style);
    },
    message.c_str(),
    WARKTerror,
    WARKTerrorStyle
    );
}

void webarkitLOGe(const std::string &message, double * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_ERROR)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(const std::string &message, float * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_ERROR)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(const std::string &message, char * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_ERROR)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(const std::string &message, const char * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_ERROR)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(const std::string &message, int  format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_ERROR)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.error(errorHead + message, style, $3);
    },
    message.c_str(),
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGw(const std::string &message) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_WARN)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.warn(infoHead + message, style);
    },
    message.c_str(),
    WARKTwarn,
    WARKTwarnStyle
    );
}

void webarkitLOGw(const std::string &message, double * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_WARN)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(const std::string &message, float * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_WARN)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(const std::string &message, char * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_WARN)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(const std::string &message, const char * format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_WARN)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message.c_str(),
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(const std::string &message, int  format) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_WARN)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.warn(errorHead + message, style, $3);
    },
    message.c_str(),
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

#if defined WEBARKIT_DEBUG

void webarkitLOGd(const std::string &message) {
    if (logSuppressed(WEBARKIT_LOG_LEVEL_DEBUG)) return;
    EM_ASM ({
        var message = UTF8ToString($0);
        var debugHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(debugHead + message, style);
    },
    message.c_str(),
    WARKTdebug,
    WARKTdebugStyle
    );
}

#else

void webarkitLOGd(const std::string &message) {
    // do nothing
}

#endif