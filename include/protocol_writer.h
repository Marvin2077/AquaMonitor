#ifndef PROTOCOL_WRITER_H
#define PROTOCOL_WRITER_H

#include <Arduino.h>

namespace Protocol {
void line(const char* msg);
void linef(const char* fmt, ...);
void error(const char* module, const char* code);
}

#endif
