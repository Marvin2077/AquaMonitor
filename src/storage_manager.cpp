#include "storage_manager.h"

Preferences StorageManager::prefs_;

/* ===== CRC 辅助函数 ===== */
uint32_t StorageManager::crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *data++;
    for (int k = 0; k < 8; ++k)
      crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
  }
  return ~crc;
}

/* ===== 通用接口 ===== */
void StorageManager::begin() {
  prefs_.begin("syscfg", false);  // 命名空间 "syscfg"，集中管理所有参数
}

void StorageManager::end() {
  prefs_.end();
}

/* ===== 温度校准参数存取 ===== */
struct CalibBlob {
  uint32_t magic   = 0x54434C42; // 'TCLB'
  uint16_t version = 1;
  uint16_t _pad    = 0;
  double a = 0, b = 1, c = 0;
  uint8_t valid = 0;
  uint8_t _rsv[7] = {0};
};

bool StorageManager::saveTempCalib(const TempService::CalibCoeff& coeff) {
  CalibBlob blob;
  blob.a = coeff.a;
  blob.b = coeff.b;
  blob.c = coeff.c;
  blob.valid = coeff.valid ? 1 : 0;

  uint32_t crc = crc32_update(0, reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));
  size_t n1 = prefs_.putBytes("temp_calib", &blob, sizeof(blob));
  size_t n2 = prefs_.putULong("temp_crc", crc);
  return (n1 == sizeof(blob));
}

bool StorageManager::loadTempCalib(TempService::CalibCoeff& out) {
  CalibBlob blob;
  uint32_t crc_stored = prefs_.getULong("temp_crc", 0);
  size_t n1 = prefs_.getBytes("temp_calib", &blob, sizeof(blob));
  if (n1 != sizeof(blob)) return false;
  if (blob.magic != 0x54434C42 || blob.version != 1) return false;

  uint32_t crc_calc = crc32_update(0, reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));
  if (crc_calc != crc_stored) return false;

  out.a = blob.a;
  out.b = blob.b;
  out.c = blob.c;
  out.valid = blob.valid != 0;
  return true;
}

bool StorageManager::clearTempCalib() {
  bool ok1 = prefs_.remove("temp_calib");
  bool ok2 = prefs_.remove("temp_crc");
  return ok1 && ok2;
}
