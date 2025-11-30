#ifndef _STORAGE_MANAGER_H
#define _STORAGE_MANAGER_H
#include "Arduino.h"
#include "Preferences.h"
extern Preferences prefs;

static const char* NVS_NS  = "dev";  // 命名空间
static const char* KEY_ID  = "id";   // key 名字

static const char* KEY_COND_K = "cond_k";      // 电导率电极常数
static const char* KEY_PH_OFF = "ph_off";      // pH Zero Offset
static const char* KEY_PH_RTIA = "ph_rtia";    // pH Rtia (Gain)
static const char* KEY_TEMP_A = "temp_a";      // 温度系数 A
static const char* KEY_TEMP_B = "temp_b";      // 温度系数 B
static const char* KEY_TEMP_C = "temp_c";      // 温度系数 C
static const char* KEY_TEMP_VALID = "temp_v";  // 温度校准是否有效

void writeDeviceID(int id);
int readDeviceID();
void ensureDeviceID();
void resetCalibrationParams();

// 1. 电导率
void saveCondParams(float k_cell);
float loadCondParams(); // 返回 K_Cell

// 2. pH
// 为了方便，我们定义一个简单的结构体来一次性返回两个值
struct PhCalibData {
    uint16_t offsetCode;
    float rtiaVal;
};
void savePhParams(uint16_t offsetCode, float rtiaVal);
PhCalibData loadPhParams();

// 3. 温度
// 对应 TempService::CalibCoeff
struct TempCalibData {
    float a, b, c;
    bool valid;
};
void saveTempParams(float a, float b, float c, bool valid);
TempCalibData loadTempParams();



#endif