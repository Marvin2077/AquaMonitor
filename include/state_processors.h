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

// pH
void processPhInit();
void processPhMeasure();
void processPhCalOffset();
void processPhCalGain();
void processPhChannel();