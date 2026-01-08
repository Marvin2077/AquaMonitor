#ifndef _BT_SERVICE_H_
#define _BT_SERVICE_H_

#include "Arduino.h"
#include "BluetoothSerial.h" // ESP32 Arduino Core 自带的库

// 声明全局的蓝牙串口对象，方便在其他地方调用（如打印日志）
extern BluetoothSerial SerialBT;

// 初始化蓝牙
void BTService_Init(const char* deviceName);

// 检查蓝牙是否有数据并读取一行
// 如果有完整的一行命令，返回 true，并将命令存入 cmdBuffer
bool BTService_ReadCommand(String &cmdBuffer);

#endif