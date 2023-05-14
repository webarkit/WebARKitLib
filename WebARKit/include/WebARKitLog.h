/*
 *  WebARKitLog.h
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

/*!
	@file log.h
	@brief Logging utilities.
	@details
        Various routines to format and redirect log output.
	@Copyright 2015-2017 Daqri, LLC.
 */

#ifndef WEBARKIT_LOG_H
#define WEBARKIT_LOG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#ifndef _WIN32 // errno is defined in stdlib.h on Windows.
#  ifdef EMSCRIPTEN // errno is not in sys/
#    include <errno.h>
#  else
#    include <sys/errno.h>
#  endif
#endif
#ifdef __ANDROID__
#  include <android/log.h>
#endif
#ifdef _WIN32
#  define ARUTIL_CALLBACK __stdcall
#else
#  define ARUTIL_CALLBACK
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WEBARKIT_LOG_LEVEL_DEBUG = 0,
    WEBARKIT_LOG_LEVEL_INFO,
    WEBARKIT_LOG_LEVEL_WARN,
    WEBARKIT_LOG_LEVEL_ERROR,
    WEBARKIT_LOG_LEVEL_REL_INFO
};
#define WEBARKIT_LOG_LEVEL_DEFAULT WEBARKIT_LOG_LEVEL_INFO

/*!
    @var int webarkitLogLevel
    @brief   Sets the severity level. Log messages below the set severity level are not logged.
	@details
        All calls to WebARKIt's logging facility include a "log level" parameter, which specifies
        the severity of the log message. (The severities are defined in &lt;WebARKitLog.h&gt;.)
        Setting this global allows for filtering of log messages. All log messages lower than
        the set level will not be logged by webarkitLog().
        Note that debug log messages created using the WEBARKIT_LOGd() macro will be logged only in
        debug builds, irrespective of the log level.
    @see webarkitLog
*/
extern int webarkitLogLevel;

/*!
    @brief   Write a string to the current logging facility.
	@details
        The default logging facility varies by platform, but on Unix-like platforms is typically
        the standard error file descriptor. However, logging may be redirected to some other
        facility by webarkitLogSetLogger.

        Newlines are not automatically appended to log output.
    @param      tag A tag to supply to an OS-specific logging function to specify the source
        of the error message. May be NULL, in which case "libwebarkit" will be used.
    @param      logLevel The severity of the log message. Defined in %lt;WebARKitLog.h&gt;.
        Log output is written to the logging facility provided the logLevel meets or
        exceeds the minimum level specified in global webarkitLogLevel.
    @param      format Log format string, in the form of printf().
    @see webarkitLogLevel
    @see webarkitLogSetLogger
*/

void webarkitLog(const char *tag, const int logLevel, const char *format, ...);
void webarkitLogv(const char *tag, const int logLevel, const char *format, va_list ap);

typedef void (ARUTIL_CALLBACK *WEBARKIT_LOG_LOGGER_CALLBACK)(const char *logMessage);

/*!
    @brief   Divert logging to a callback, or revert to default logging.
	@details
        The default logging facility varies by platform, but on Unix-like platforms is typically
        the standard error file descriptor. However, logging may be redirected to some other
        facility by this function.
    @param      callback The function which will be called with the log output, or NULL to
        cancel redirection.
    @param      callBackOnlyIfOnSameThread If non-zero, then the callback will only be called
        if the call to webarkitLog is made on the same thread as the thread which called this function,
        and if the webarkitLog call is made on a different thread, log output will be buffered until
        the next call to webarkitLog on the original thread.

        The purpose of this is to prevent logging from secondary threads in cases where the
        callback model of the target platform precludes this.
    @see webarkitLog
*/
void webarkitLogSetLogger(WEBARKIT_LOG_LOGGER_CALLBACK callback, int callBackOnlyIfOnSameThread);

#ifdef WEBARKIT_DEBUG
#  define WEBARKIT_LOGd(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#  define WEBARKIT_LOGd(...)
#endif
#define WEBARKIT_LOGi(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_INFO, __VA_ARGS__)
#define WEBARKIT_LOGw(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_WARN, __VA_ARGS__)
#define WEBARKIT_LOGe(...) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_ERROR, __VA_ARGS__)
#define WEBARKIT_LOGperror(s) webarkitLog(NULL, WEBARKIT_LOG_LEVEL_ERROR, ((s != NULL) ? "%s: %s\n" : "%s%s\n"), ((s != NULL) ? s : ""), strerror(errno))

#ifdef __ANDROID__
#  define WEBARKIT_LOG(...)  __android_log_print(ANDROID_LOG_INFO, "WebARKit", __VA_ARGS__)
#else
#  define WEBARKIT_LOG(...)  printf(__VA_ARGS__)
#endif

#ifdef __cplusplus
}
#endif
#endif //#ifndef WEBARKIT_LOG_H
