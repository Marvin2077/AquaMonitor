#include "storage_manager.h"
#include "debug_log.h"
#include "protocol_writer.h"
#include "serial_command_reader.h"
Preferences prefs;
#define DEFAULT_COND_K  1.0f         // 默认电极常数
#define DEFAULT_PH_OFF  32768        // 默认中点 (ADC半量程)
#define DEFAULT_PH_RTIA 170000.0f       // 默认反馈电阻 (根据你的硬件填)
static constexpr uint8_t CALIB_STORAGE_VERSION = 1;

static bool versionValid(Preferences& p, const char* versionKey, bool legacyPresent) {
    uint8_t version = p.getUChar(versionKey, 0);
    return version == CALIB_STORAGE_VERSION || (version == 0 && legacyPresent);
}

void writeDeviceID(int id)
{
    prefs.begin(NVS_NS, false);
    prefs.putInt(KEY_ID, id);
    prefs.end();
}

int readDeviceID()
{
    int defaultId = -1;
    prefs.begin(NVS_NS, true);
    int id = prefs.getInt(KEY_ID,defaultId);
    prefs.end();
    return id;
}

void ensureDeviceID() {
  int Deviceid;
  Deviceid = readDeviceID();

  if (Deviceid != -1) {
    LOG_DEBUG_PRINT("device id = ");
    LOG_DEBUG_PRINTLN(Deviceid);
    return;
  }

  Protocol::error("DEVICE_ID", "NOT_SET");
  LOG_DEBUG_PRINTLN("Please send via Serial: id <number> e.g. id 1");

  while (true) {
    String cmd;
    SerialReadResult result = readSerialCommandLine(cmd);
    if (result == SerialReadResult::OVERFLOW) {
      Protocol::error("SERIAL", "LINE_TOO_LONG");
      continue;
    }
    if (result == SerialReadResult::LINE_READY) {

      // 允许格式: "id 3" / "id:3" / "id=3"
      if (cmd.startsWith("id")) {
        cmd.replace("id", "");
        cmd.replace(":", " ");
        cmd.replace("=", " ");
        cmd.trim();

        int newId = cmd.toInt();

        // 你可以按需要调整范围
        if (newId > 0 && newId < 1000000) {
          writeDeviceID(newId);
          Deviceid = newId;
          Protocol::linef("$SYS,DEVICE_ID,SAVED,%d*", Deviceid);
          break;
        }
      }

      Protocol::error("DEVICE_ID", "INVALID");
      LOG_DEBUG_PRINTLN("Invalid ID. Use: id <positive int> e.g. id 1");
    }

    delay(10);
    yield();
  }
}

// 1. 电导率存取
void saveCondParams(float k_cell) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(KEY_COND_K, k_cell);
    prefs.putBool(KEY_COND_K_VALID, true);
    prefs.putUChar(KEY_COND_K_VERSION, CALIB_STORAGE_VERSION);
    LOG_DEBUG_PRINTF("[Storage] Saved Cond K: %.4f\n", k_cell);
    prefs.end();
}

float loadCondParams() {
    prefs.begin(NVS_NS, true);
    bool legacyPresent = prefs.isKey(KEY_COND_K);
    bool valid = prefs.getBool(KEY_COND_K_VALID, legacyPresent);
    float k = DEFAULT_COND_K;
    if (valid && versionValid(prefs, KEY_COND_K_VERSION, legacyPresent)) {
        float stored = prefs.getFloat(KEY_COND_K, DEFAULT_COND_K);
        if (stored > 0.0f && stored < 100.0f) {
            k = stored;
        }
    }
    prefs.end();
    return k;
}

// 2. pH 存取
void savePhParams(uint16_t offsetCode, float rtiaVal) {
    prefs.begin(NVS_NS, false);
    prefs.putUShort(KEY_PH_OFF, offsetCode); // UShort = uint16_t
    prefs.putFloat(KEY_PH_RTIA, rtiaVal);
    prefs.putBool(KEY_PH_VALID, true);
    prefs.putUChar(KEY_PH_VERSION, CALIB_STORAGE_VERSION);
    LOG_DEBUG_PRINTF("[Storage] Saved pH Off: %d, Rtia: %.2f\n", offsetCode, rtiaVal);
    prefs.end();
}

PhCalibData loadPhParams() {
    prefs.begin(NVS_NS, true);
    PhCalibData data;
    data.offsetCode = DEFAULT_PH_OFF;
    data.rtiaVal = DEFAULT_PH_RTIA;
    bool legacyPresent = prefs.isKey(KEY_PH_OFF) && prefs.isKey(KEY_PH_RTIA);
    bool valid = prefs.getBool(KEY_PH_VALID, legacyPresent);
    if (valid && versionValid(prefs, KEY_PH_VERSION, legacyPresent)) {
        float rtia = prefs.getFloat(KEY_PH_RTIA, DEFAULT_PH_RTIA);
        if (rtia > 0.0f && rtia < 10000000.0f) {
            data.offsetCode = prefs.getUShort(KEY_PH_OFF, DEFAULT_PH_OFF);
            data.rtiaVal = rtia;
        }
    }
    prefs.end();
    return data;
}

// 3. 温度存取
void saveTempParams(float a, float b, float c, bool valid) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(KEY_TEMP_A, a);
    prefs.putFloat(KEY_TEMP_B, b);
    prefs.putFloat(KEY_TEMP_C, c);
    prefs.putBool(KEY_TEMP_VALID, valid);
    prefs.putUChar(KEY_TEMP_VERSION, CALIB_STORAGE_VERSION);
    LOG_DEBUG_PRINTLN("[Storage] Saved Temp Coeffs.");
    prefs.end();
}

TempCalibData loadTempParams() {
    prefs.begin(NVS_NS, true);
    TempCalibData data;
    data.a = prefs.getFloat(KEY_TEMP_A, 0.0f);
    data.b = prefs.getFloat(KEY_TEMP_B, 0.0f);
    data.c = prefs.getFloat(KEY_TEMP_C, 0.0f);
    bool legacyPresent = prefs.isKey(KEY_TEMP_A) && prefs.isKey(KEY_TEMP_B) && prefs.isKey(KEY_TEMP_C);
    bool valid = prefs.getBool(KEY_TEMP_VALID, false);
    data.valid = valid && versionValid(prefs, KEY_TEMP_VERSION, legacyPresent);
    prefs.end();
    return data;
}

// 4. 电导率三点校准
void saveCondCalib(float a, float b, float c, bool valid) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(KEY_COND_CAL_A, a);
    prefs.putFloat(KEY_COND_CAL_B, b);
    prefs.putFloat(KEY_COND_CAL_C, c);
    prefs.putBool(KEY_COND_CAL_VALID, valid);
    prefs.putUChar(KEY_COND_CAL_VERSION, CALIB_STORAGE_VERSION);
    LOG_DEBUG_PRINTLN("[Storage] Saved Cond Calib Coeffs.");
    prefs.end();
}

CondCalibData loadCondCalib() {
    prefs.begin(NVS_NS, true);
    CondCalibData data;
    data.a = prefs.getFloat(KEY_COND_CAL_A, 0.0f);
    data.b = prefs.getFloat(KEY_COND_CAL_B, 1.0f);
    data.c = prefs.getFloat(KEY_COND_CAL_C, 0.0f);
    bool legacyPresent = prefs.isKey(KEY_COND_CAL_A) && prefs.isKey(KEY_COND_CAL_B) && prefs.isKey(KEY_COND_CAL_C);
    bool valid = prefs.getBool(KEY_COND_CAL_VALID, false);
    data.valid = valid && versionValid(prefs, KEY_COND_CAL_VERSION, legacyPresent);
    prefs.end();
    return data;
}

void resetCalibrationParams() {
    prefs.begin(NVS_NS, false); // false = read/write

    // 1. 删除电导率参数
    prefs.remove(KEY_COND_K);
    prefs.remove(KEY_COND_K_VALID);
    prefs.remove(KEY_COND_K_VERSION);

    // 2. 删除 pH 参数
    prefs.remove(KEY_PH_OFF);
    prefs.remove(KEY_PH_RTIA);
    prefs.remove(KEY_PH_VALID);
    prefs.remove(KEY_PH_VERSION);

    // 3. 删除温度参数
    prefs.remove(KEY_TEMP_A);
    prefs.remove(KEY_TEMP_B);
    prefs.remove(KEY_TEMP_C);
    prefs.remove(KEY_TEMP_VALID);
    prefs.remove(KEY_TEMP_VERSION);

    // 4. 删除电导率三点校准参数
    prefs.remove(KEY_COND_CAL_A);
    prefs.remove(KEY_COND_CAL_B);
    prefs.remove(KEY_COND_CAL_C);
    prefs.remove(KEY_COND_CAL_VALID);
    prefs.remove(KEY_COND_CAL_VERSION);

    LOG_DEBUG_PRINTLN("[Storage] All calibration params deleted (reset to defaults).");
    prefs.end();
}
