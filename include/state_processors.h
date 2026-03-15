#pragma once

// 温度
void processTempMeasure();
void processTempCalP1();
void processTempCalP2();
void processTempCalP3();
void processTempSaveCal();
void processTempResetCal();
void processTempResistance();

// 电导率
void processCondInit();
void processCondMeasure();
void processCondSweep();
void processCondCal();
void processCondCalPoint();   // P1/P2/P3 共用同一个函数体
void processCondSaveCal();
void processCondResetCal();

// pH
void processPhInit();
void processPhMeasure();
void processPhCalOffset();
void processPhCalGain();
void processPhChannel();