#ifndef _MUX_IFACE_H_
#define _MUX_IFACE_H_
#include "Arduino.h"
#include "stdio.h"
#include "string.h"
void ChooseSenesingChannel(int channel);
void ChooseISFETChannel(int channel);
/* ====== ISFET MUX 特定引脚 ====== */
static const int ISFET_MUX_ADDR0 =  32; // ISFET_MUX_ADDR0引脚
static const int ISFET_MUX_ADDR1 =  33; // ISFET_MUX_ADDR1引脚
static const int ISFET_MUX_ADDR2 =  25; // ISFET_MUX_ADDR2引脚
/* ====== Channel MUX 特定引脚 ====== */
static const int CHANNEL_MUX_ADDR0 =  17; // CHANNEL_MUX_ADDR0 复位引脚
static const int CHANNEL_MUX_ADDR1 =  18; // CHANNEL_MUX_ADDR1 复位引脚
#endif