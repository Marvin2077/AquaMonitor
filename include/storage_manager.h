#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "temp_service.h"  // 需要用到 CalibCoeff 结构

class StorageManager {
public:
  static void begin();          // 初始化 Preferences
  static void end();            // 结束（可选）

  // === 温度校准 ===
  static bool saveTempCalib(const TempService::CalibCoeff& coeff);
  static bool loadTempCalib(TempService::CalibCoeff& out);
  static bool clearTempCalib();

private:
  static Preferences prefs_;
  static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len);
};
