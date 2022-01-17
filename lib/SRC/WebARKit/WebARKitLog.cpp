#include <WebARKit/WebARKitLog.h>

const char * WARKTinfo = "%cℹ️[webarkit-info:]";
const char * WARKTinfoStyle = "color: #664400; background-color: #ffff99; border-radius: 4px; padding: 2px";
const char * WARKTerror = "%c🚩[webarkit-error:]";
const char * WARKTerrorStyle = "color: #ffffff; background-color: #ff0101; border-radius: 4px; padding: 2px";
const char * WARKTwarn = "%c⚠️[webarkit-warn:]";
const char * WARKTwarnStyle = "color: #774400; background-color: #ffff99; border-radius: 4px; padding: 2px";

void webarkitLOGi(char* message) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(infoHead + message, style);
    },
    message,
    WARKTinfo,
    WARKTinfoStyle
    );
}

void webarkitLOGi(char* message, double * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message,
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(char* message, float * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message,
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(char* message, char * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message,
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(char* message, const char * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.log(infoHead + message, style, format);
    },
    message,
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGi(char* message, int  format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(infoHead + message, style, $3);
    },
    message,
    WARKTinfo,
    WARKTinfoStyle,
    format
    );
}

void webarkitLOGe(char* message) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.error(errorHead + message, style);
    },
    message,
    WARKTerror,
    WARKTerrorStyle
    );
}

void webarkitLOGe(char* message, double * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message,
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(char* message, float * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message,
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(char* message, char * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message,
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(char* message, const char * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.error(errorHead + message, style, format);
    },
    message,
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGe(char* message, int  format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.error(errorHead + message, style, $3);
    },
    message,
    WARKTerror,
    WARKTerrorStyle,
    format
    );
}

void webarkitLOGw(char* message) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var infoHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.warn(infoHead + message, style);
    },
    message,
    WARKTwarn,
    WARKTwarnStyle
    );
}

void webarkitLOGw(char* message, double * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message,
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(char* message, float * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message,
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(char* message, char * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message,
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(char* message, const char * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        var format = UTF8ToString($3);
        console.warn(errorHead + message, style, format);
    },
    message,
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}

void webarkitLOGw(char* message, int  format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var errorHead =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.warn(errorHead + message, style, $3);
    },
    message,
    WARKTwarn,
    WARKTwarnStyle,
    format
    );
}