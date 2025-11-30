#include "storage_manager.h"
Preferences prefs;
#define DEFAULT_COND_K  1.0f         // 默认电极常数
#define DEFAULT_PH_OFF  32768        // 默认中点 (ADC半量程)
#define DEFAULT_PH_RTIA 1000.0f       // 默认反馈电阻 (根据你的硬件填)
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
    Serial.print("device id = ");
    Serial.println(Deviceid);
    return;
  }

  Serial.println("Device ID not set!");
  Serial.println("Please send via Serial:  id <number>   e.g.  id 1");

  while (true) {
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

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
          Serial.print("Device ID saved: ");
          Serial.println(Deviceid);
          break;
        }
      }

      Serial.println("Invalid ID. Use: id <positive int>  e.g. id 1");
    }

    delay(10);
    yield();
  }
}

// 1. 电导率存取
void saveCondParams(float k_cell) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat(KEY_COND_K, k_cell);
    Serial.printf("[Storage] Saved Cond K: %.4f\n", k_cell);
    prefs.end();
}

float loadCondParams() {
    prefs.begin(NVS_NS, true);
    float k = prefs.getFloat(KEY_COND_K, DEFAULT_COND_K); // 如果没存过，返回默认值
    prefs.end();
    return k;
}

// 2. pH 存取
void savePhParams(uint16_t offsetCode, float rtiaVal) {
    prefs.begin(NVS_NS, false);
    prefs.putUShort(KEY_PH_OFF, offsetCode); // UShort = uint16_t
    prefs.putFloat(KEY_PH_RTIA, rtiaVal);
    Serial.printf("[Storage] Saved pH Off: %d, Rtia: %.2f\n", offsetCode, rtiaVal);
    prefs.end();
}

PhCalibData loadPhParams() {
    prefs.begin(NVS_NS, true);
    PhCalibData data;
    data.offsetCode = prefs.getUShort(KEY_PH_OFF, DEFAULT_PH_OFF);
    data.rtiaVal = prefs.getFloat(KEY_PH_RTIA, DEFAULT_PH_RTIA);
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
    Serial.println("[Storage] Saved Temp Coeffs.");
    prefs.end();
}

TempCalibData loadTempParams() {
    prefs.begin(NVS_NS, true);
    TempCalibData data;
    data.a = prefs.getFloat(KEY_TEMP_A, 0.0f);
    data.b = prefs.getFloat(KEY_TEMP_B, 0.0f);
    data.c = prefs.getFloat(KEY_TEMP_C, 0.0f);
    data.valid = prefs.getBool(KEY_TEMP_VALID, false);
    prefs.end();
    return data;
}

void resetCalibrationParams() {
    prefs.begin(NVS_NS, false); // false = read/write

    // 1. 删除电导率参数
    prefs.remove(KEY_COND_K);
    
    // 2. 删除 pH 参数
    prefs.remove(KEY_PH_OFF);
    prefs.remove(KEY_PH_RTIA);
    
    // 3. 删除温度参数
    prefs.remove(KEY_TEMP_A);
    prefs.remove(KEY_TEMP_B);
    prefs.remove(KEY_TEMP_C);
    prefs.remove(KEY_TEMP_VALID);

    Serial.println("[Storage] All calibration params deleted (reset to defaults).");
    prefs.end();
}