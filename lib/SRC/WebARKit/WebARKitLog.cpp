#include <WebARKit/WebARKitLog.h>

const char * WARKTinfo = "%cℹ️[webarkit-info:]";
const char * WARKTinfoStyle = "color: #664400; background-color: #ffff99; border-radius: 4px; padding: 2px";
const char * WARKTerror = "%c🚩[webarkit-error:]";
const char * WARKTerrorStyle = "color: #ffffff; background-color: #ff0101; border-radius: 4px; padding: 2px";
const char * WARKTwarn = "%c⚠️[webarkit-warn:]";
const char * WARKTwarnStyle = "color: #774400; background-color: #ffff99; border-radius: 4px; padding: 2px";

void webarkitLOGi(const std::string &message) {
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