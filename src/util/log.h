/*
 * log.h
 *
 *  Created on: 20.08.2017
 *      Author: rYan
 */

#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <system_error>

#define LogDebug(fmt, ...) \
        do { if (1) fprintf(stderr, "Debug %s:%d:%s(): " fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

#define LogError(fmt, ...) \
        do { if (1) fprintf(stderr, "Error %s:%d:%s(): " fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

#define LogWarn(fmt, ...) \
        do { if (1) fprintf(stderr, "Warn  %s:%d:%s(): " fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

#define LogInfo(fmt, ...) \
        do { if (1) fprintf(stderr, "Info  %s:%d:%s(): " fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

#define LogTrace(fmt, ...) \
        do { if (0) fprintf(stderr, "Trace %s:%d:%s(): " fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

class mmal_error : public std::runtime_error
{
public:
    mmal_error(int code)
        :std::runtime_error(std::string("MMAL Error ") + std::to_string(code)) {}

    virtual ~mmal_error() {};
};
