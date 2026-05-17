#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <Arduino.h>

#ifndef FW_DEBUG_LOG
#define FW_DEBUG_LOG 0
#endif

#if FW_DEBUG_LOG
#define LOG_DEBUG_PRINT(msg) Serial.print(msg)
#define LOG_DEBUG_PRINTLN(msg) Serial.println(msg)
#define LOG_DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define LOG_DEBUG_PRINT(msg) ((void)0)
#define LOG_DEBUG_PRINTLN(msg) ((void)0)
#define LOG_DEBUG_PRINTF(...) ((void)0)
#endif

#endif
