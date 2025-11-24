#ifndef _STORAGE_MANAGER_H
#define _STORAGE_MANAGER_H
#include "Arduino.h"
#include "Preferences.h"
extern Preferences prefs;

static const char* NVS_NS  = "dev";  // 命名空间
static const char* KEY_ID  = "id";   // key 名字
void writeDeviceID(int id);
int readDeviceID();
void ensureDeviceID();

#endif