#include "protocol_writer.h"
#include <stdarg.h>
#include <stdio.h>

namespace Protocol {

void line(const char* msg) {
    Serial.println(msg);
}

void linef(const char* fmt, ...) {
    char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.println(buf);
}

void error(const char* module, const char* code) {
    linef("$ERR,%s,%s*", module, code);
}

}
