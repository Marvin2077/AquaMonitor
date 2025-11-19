#include "mux_iface.h"

/*
static const int CHANNEL_MUX_ADDR0 =  17; // CHANNEL_MUX_ADDR0 复位引脚
static const int CHANNEL_MUX_ADDR1 =  18; // CHANNEL_MUX_ADDR1 复位引脚
*/
void ChooseSenesingChannel(int channel)
{
    //ADDR0-IO17
    //ADDR1-IO18
    switch(channel)
    {
     case 1:
        //ADDR1 - 0 ADDR0 - 0  -> S1A & S1B
        //Impedance
        digitalWrite(CHANNEL_MUX_ADDR1,LOW);
        digitalWrite(CHANNEL_MUX_ADDR0,LOW);
        break;
    case 2:
        //ADDR1 - 0 ADDR0 - 1 -> S2A & S2B
        digitalWrite(CHANNEL_MUX_ADDR1,LOW);
        digitalWrite(CHANNEL_MUX_ADDR0,HIGH);
        //余氯
        break;
    case 3:
        //ADDR1 - 1 ADDR0 - 0 -> S1A & S1B
        digitalWrite(CHANNEL_MUX_ADDR1,HIGH);
        digitalWrite(CHANNEL_MUX_ADDR0,LOW);
        break;
    default:
        //ADDR0 - 0 ADDR1 - 0 -> S1A & S1B
        digitalWrite(17|18,LOW);
        break;
    }

}

/*
static const int ISFET_MUX_ADDR0 =  32; // ISFET_MUX_ADDR0引脚
static const int ISFET_MUX_ADDR1 =  33; // ISFET_MUX_ADDR1引脚
static const int ISFET_MUX_ADDR2 =  25; // ISFET_MUX_ADDR2引脚
*/
void ChooseISFETChannel(int channel)
{
    //ISFET-ADDR0-IO32
    //ISFET-ADDR1-IO33
    //ISFET-ADDR2-IO25
    //ISFET Channel
  switch(channel)
    {
     case 1:
        //ADDR2 - 0 ADDR1 - 0  ADDR0 - 0-> S1
        digitalWrite(ISFET_MUX_ADDR2,LOW);
        digitalWrite(ISFET_MUX_ADDR1,LOW);
        digitalWrite(ISFET_MUX_ADDR0,LOW);
        break;
    case 2:
        //ADDR2 - 0 ADDR1 - 0 ADDR0 - 1 -> S2
        digitalWrite(ISFET_MUX_ADDR2,LOW);
        digitalWrite(ISFET_MUX_ADDR1,LOW);
        digitalWrite(ISFET_MUX_ADDR0,HIGH);
        //余氯
        break;
    case 3:
        //ADDR2 - 0 ADDR1 - 1 ADDR0 - 0 -> S3
        digitalWrite(ISFET_MUX_ADDR2,LOW);
        digitalWrite(ISFET_MUX_ADDR1,HIGH);
        digitalWrite(ISFET_MUX_ADDR0,LOW);
        break;
    case 4:
        //ADDR2 - 0 ADDR1 - 1 ADDR0 - 1 -> S4
        digitalWrite(ISFET_MUX_ADDR2,LOW);
        digitalWrite(ISFET_MUX_ADDR1,HIGH);
        digitalWrite(ISFET_MUX_ADDR0,HIGH);
        break;
     case 5:
        //ADDR2 - 1 ADDR1 - 0 ADDR0 - 0 -> S5
        digitalWrite(ISFET_MUX_ADDR2,HIGH);
        digitalWrite(ISFET_MUX_ADDR1,LOW);
        digitalWrite(ISFET_MUX_ADDR0,LOW);
        break;
    case 6:
        //ADDR2 - 1 ADDR1 - 0 ADDR0 - 1 -> S6
        digitalWrite(ISFET_MUX_ADDR2,HIGH);
        digitalWrite(ISFET_MUX_ADDR1,LOW);
        digitalWrite(ISFET_MUX_ADDR0,HIGH);
        //余氯
        break;
    case 7:
        //ADDR2 - 1 ADDR1 - 1 ADDR0 - 0 -> S7
        digitalWrite(ISFET_MUX_ADDR2,HIGH);
        digitalWrite(ISFET_MUX_ADDR1,HIGH);
        digitalWrite(ISFET_MUX_ADDR0,LOW);
        break;
    case 8:
        //ADDR2 - 1 ADDR1 - 1 ADDR0 - 1 -> S8
        digitalWrite(ISFET_MUX_ADDR2,HIGH);
        digitalWrite(ISFET_MUX_ADDR1,HIGH);
        digitalWrite(ISFET_MUX_ADDR0,HIGH);
        break;
    default:
        //ADDR2 - 0 ADDR1 - 0 ADDR0 - 0 -> S1
        digitalWrite(ISFET_MUX_ADDR2,LOW);
        digitalWrite(ISFET_MUX_ADDR1,LOW);
        digitalWrite(ISFET_MUX_ADDR0,LOW);
        break;
    }
}