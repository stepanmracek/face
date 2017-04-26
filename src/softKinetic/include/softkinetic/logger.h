#ifndef FACELIB_LOGGER_H
#define FACELIB_LOGGER_H

#ifdef TBS_DEVICE
#include "TBS/Log.h"
#define LOG_TRACE(msg) LTRACE("DS325") << msg << LE
#define LOG_DEBUG(msg) LDEBUG("DS325") << msg << LE
#define LOG_ERROR(msg) LERROR("DS325") << msg << LE
#else
#include <iostream>
#define LOG_TRACE(msg) std::cout << msg << std::endl;
#define LOG_DEBUG(msg) std::cout << msg << std::endl;
#define LOG_ERROR(msg) std::cout << msg << std::endl;
#endif

#endif // LOGGER_H
