#include <WebARKit/WebARKitLog.h>

const char * WARKTinfo = "%c[webarkit-info]: ";
const char * WARKTstyle = "color: #664400; background-color: #ffff99; border-radius: 4px; padding: 2px";

void webarkitLOGi(char* message) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var info =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(info + message, style);
    },
    message,
    WARKTinfo,
    WARKTstyle
    );
}

void webarkitLOGi(char* message, double * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var info =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(info + message, style, $3);
    },
    message,
    WARKTinfo,
    WARKTstyle,
    format
    );
}

void webarkitLOGi(char* message, float * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var info =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(info + message, style, $3);
    },
    message,
    WARKTinfo,
    WARKTstyle,
    format
    );
}

void webarkitLOGi(char* message, char * format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var info =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(info + message, style, $3);
    },
    message,
    WARKTinfo,
    WARKTstyle,
    format
    );
}

void webarkitLOGi(char* message, int  format) {
    EM_ASM ({
        var message = UTF8ToString($0);
        var info =  UTF8ToString($1);
        var style = UTF8ToString($2);
        console.log(info + message, style, $3);
    },
    message,
    WARKTinfo,
    WARKTstyle,
    format
    );
}