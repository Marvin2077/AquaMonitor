#include "storage_manager.h"
Preferences prefs;
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

    delay(10);   // 喂狗/让出CPU，避免WDT
    yield();
  }
}
