/*
 *  WebARKitLog.cpp
 *  WebARKit
 *
 *  This file is part of WebARKit.
 *
 *  WebARKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  WebARKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with WebARKit.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2015-2016 Daqri, LLC.
 *  Copyright 2003-2015 ARToolworks, Inc.
 *  Copyright 2023 WebARKit.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb, Walter Perdan
 *
 *  This code was taken from Artoolkit5 https://github.com/artoolkitx/artoolkit5
 *  with small modifications to adapt to existing WebARKIt code
 *
 */

#include <WebARKitLog.h>

#ifndef _WIN32
#  include <pthread.h> // pthread_self(), pthread_equal()
#  ifdef __APPLE__
#    include <os/log.h>
#  endif
#else
#  include <Windows.h>
#  define snprintf _snprintf
#endif

//
// Global required for logging functions.
//
int webarkitLogLevel = WEBARKIT_LOG_LEVEL_DEFAULT;
static WEBARKIT_LOG_LOGGER_CALLBACK webarkitLogLoggerCallback = NULL;
static int webarkitLogLoggerCallBackOnlyIfOnSameThread = 0;
#ifndef _WIN32
static pthread_t webarkitLogLoggerThread;
#else
static DWORD webarkitLogLoggerThreadID;
#endif
#define WEBARKIT_LOG_WRONG_THREAD_BUFFER_SIZE 4096
static char *webarkitLogWrongThreadBuffer = NULL;
static size_t webarkitLogWrongThreadBufferSize = 0;
static size_t webarkitLogWrongThreadBufferCount = 0;


void webarkitLogSetLogger(WEBARKIT_LOG_LOGGER_CALLBACK callback, int callBackOnlyIfOnSameThread)
{
    webarkitLogLoggerCallback = callback;
    webarkitLogLoggerCallBackOnlyIfOnSameThread = callBackOnlyIfOnSameThread;
    if (callback && callBackOnlyIfOnSameThread) {
#ifndef _WIN32
        webarkitLogLoggerThread = pthread_self();
#else
        webarkitLogLoggerThreadID = GetCurrentThreadId();
#endif
		if (!webarkitLogWrongThreadBuffer) {
            if ((webarkitLogWrongThreadBuffer = static_cast<char*>(malloc(sizeof(char) * WEBARKIT_LOG_WRONG_THREAD_BUFFER_SIZE)))) {
                webarkitLogWrongThreadBufferSize = WEBARKIT_LOG_WRONG_THREAD_BUFFER_SIZE;
            }
		}
    } else {
		if (webarkitLogWrongThreadBuffer) {
			free(webarkitLogWrongThreadBuffer);
			webarkitLogWrongThreadBuffer = NULL;
			webarkitLogWrongThreadBufferSize = 0;
		}
	}
}

void webarkitLog(const char *tag, const int logLevel, const char *format, ...)
{
    if (logLevel < webarkitLogLevel) return;
    if (!format || !format[0]) return;
    
    va_list ap;
    va_start(ap, format);
    webarkitLogv(tag, logLevel, format, ap);
    va_end(ap);
}

void webarkitLogv(const char *tag, const int logLevel, const char *format, va_list ap)
{
    va_list ap2;
    char *buf = NULL;
    size_t len;
    const char *logLevelStrings[] = {
        "debug",
        "info",
        "warning",
        "error"
    };
    const size_t logLevelStringsCount = (sizeof(logLevelStrings)/sizeof(logLevelStrings[0]));
    size_t logLevelStringLen;

    if (logLevel < webarkitLogLevel) return;
    if (!format || !format[0]) return;

    // Count length required to unpack varargs.
    va_copy(ap2, ap);
#ifdef _WIN32
    len = _vscprintf(format, ap);
#else
    len = vsnprintf(NULL, 0, format, ap2);
#endif
    va_end(ap2);
    if (len < 1) return;
    
    // Add characters required for logLevelString.
    if (logLevel >= 0 && logLevel < (int)logLevelStringsCount) {
        logLevelStringLen = 3 + strlen(logLevelStrings[logLevel]); // +3 for brackets and a space, e.g. "[debug] ".
    } else {
        logLevelStringLen = 0;
    }
    
    buf = (char *)malloc((logLevelStringLen + len + 1) * sizeof(char)); // +1 for nul-term.
    
    if (logLevelStringLen > 0) {
        snprintf(buf, logLevelStringLen + 1, "[%s] ", logLevelStrings[logLevel]);
    }
    
    vsnprintf(buf + logLevelStringLen, len + 1, format, ap);
    len += logLevelStringLen;
    
    if (webarkitLogLoggerCallback) {
        
        if (!webarkitLogLoggerCallBackOnlyIfOnSameThread) {
            (*webarkitLogLoggerCallback)(buf);
        } else {
#ifndef _WIN32
            if (!pthread_equal(pthread_self(), webarkitLogLoggerThread))
#else
            if (GetCurrentThreadId() != webarkitLogLoggerThreadID)
#endif
            {
                // On non-log thread, put it into buffer if we can.
                if (webarkitLogWrongThreadBuffer && (webarkitLogWrongThreadBufferCount < webarkitLogWrongThreadBufferSize)) {
                    if (len <= (webarkitLogWrongThreadBufferSize - (webarkitLogWrongThreadBufferCount + 4))) { // +4 to reserve space for "...\0".
                        strncpy(&webarkitLogWrongThreadBuffer[webarkitLogWrongThreadBufferCount], buf, len + 1);
                        webarkitLogWrongThreadBufferCount += len;
                    } else {
                        strncpy(&webarkitLogWrongThreadBuffer[webarkitLogWrongThreadBufferCount], "...", 4);
                        webarkitLogWrongThreadBufferCount = webarkitLogWrongThreadBufferSize; // Mark buffer as full.
                    }
                }
            } else {
                // On log thread, print buffer if anything was in it, then the current message.
                if (webarkitLogWrongThreadBufferCount > 0) {
                    (*webarkitLogLoggerCallback)(webarkitLogWrongThreadBuffer);
                    webarkitLogWrongThreadBufferCount = 0;
                }
                (*webarkitLogLoggerCallback)(buf);
            }
        }
        
    } else {
#if defined(__ANDROID__)
        int logLevelA;
        switch (logLevel) {
            case WEBARKIT_LOG_LEVEL_REL_INFO:         logLevelA = ANDROID_LOG_ERROR; break;
            case WEBARKIT_LOG_LEVEL_ERROR:            logLevelA = ANDROID_LOG_ERROR; break;
            case WEBARKIT_LOG_LEVEL_WARN:             logLevelA = ANDROID_LOG_WARN;  break;
            case WEBARKIT_LOG_LEVEL_INFO:             logLevelA = ANDROID_LOG_INFO;  break;
            case WEBARKIT_LOG_LEVEL_DEBUG: default:   logLevelA = ANDROID_LOG_DEBUG; break;
        }
        __android_log_write(logLevelA, (tag ? tag : "libAR"), buf);
        //#elif defined(_WINRT)
        //            OutputDebugStringA(buf);
#elif defined(__APPLE__)
        if (os_log_create == NULL) { // os_log only available macOS 10.12 / iOS 10.0 and later.
            fprintf(stderr, "%s", buf);
        } else {
            os_log_type_t type;
            switch (logLevel) {
                case WEBARKIT_LOG_LEVEL_REL_INFO:         type = OS_LOG_TYPE_DEFAULT; break;
                case WEBARKIT_LOG_LEVEL_ERROR:            type = OS_LOG_TYPE_ERROR; break;
                case WEBARKIT_LOG_LEVEL_WARN:             type = OS_LOG_TYPE_DEFAULT;  break;
                case WEBARKIT_LOG_LEVEL_INFO:             type = OS_LOG_TYPE_INFO;  break;
                case WEBARKIT_LOG_LEVEL_DEBUG: default:   type = OS_LOG_TYPE_DEBUG; break;
            }
            os_log_with_type(OS_LOG_DEFAULT, type, "%{public}s", buf);
        }
#else
        fprintf(stderr, "%s", buf);
#endif
    }
    free(buf);
}
