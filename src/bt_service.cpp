#include "bt_service.h"

BluetoothSerial SerialBT;

void BTService_Init(const char* deviceName)
{
    if(!SerialBT.begin(deviceName)) {
        Serial.println("An error occurred initializing Bluetooth");
    } else {
        Serial.printf("Bluetooth initialized! Device Name: %s\n", deviceName);
        Serial.println("You can now pair with it and use a Serial Terminal.");
    }
}

bool BTService_ReadCommand(String &cmdBuffer)
{
    if (SerialBT.available()) {
        // 读取直到遇到换行符
        // 注意：串口助手发送时必须加回车换行符
        cmdBuffer = SerialBT.readStringUntil('\n');
        cmdBuffer.trim(); // 去除首尾空白符（回车、换行、空格）
        
        if (cmdBuffer.length() > 0) {
            return true;
        }
    }
    return false;
}