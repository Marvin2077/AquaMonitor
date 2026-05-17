#ifndef SERIAL_COMMAND_READER_H
#define SERIAL_COMMAND_READER_H

#include <Arduino.h>

enum class SerialReadResult {
    NONE,
    LINE_READY,
    OVERFLOW
};

SerialReadResult readSerialCommandLine(String& out);

#endif
