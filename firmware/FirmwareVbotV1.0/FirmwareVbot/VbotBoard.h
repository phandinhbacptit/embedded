#ifndef VbotBoard_H
#define VbotBoard_H

#include "Arduino.h"   

// Supported Modules drive needs to be added here
#include "VnPort.h"
#include "VnUltrasonicSensor.h"
#include "VnDCMotor.h"
#include "VbotDCMotor.h"
#include "VnBuzzer.h"
#include "VnLineFollower.h"
#include "VnLightSensor.h"
#include "VnServo.h"
#include "VnBle.h"
//#include "VnColorSensor.h"
#include "VnIrRemote.h"
#include "VnButton.h"
#include "VnRGB.h"

/*Define for DC motor*/
#define LEFT            1
#define RIGHT           2
#define FORWARD         3
#define BACKWARD        4
/*Define for buzzer*/
#define NOTE_B0         31
#define NOTE_C1         33
#define NOTE_CS1        35
#define NOTE_D1         37
#define NOTE_DS1        39
#define NOTE_E1         41
#define NOTE_F1         44
#define NOTE_FS1        46
#define NOTE_G1         49
#define NOTE_GS1        52
#define NOTE_A1         55
#define NOTE_AS1        58
#define NOTE_B1         62
#define NOTE_C2         65
#define NOTE_CS2        69
#define NOTE_D2         73
#define NOTE_DS2        78
#define NOTE_E2         82
#define NOTE_F2         87
#define NOTE_FS2        93
#define NOTE_G2         98
#define NOTE_GS2        104
#define NOTE_A2         110
#define NOTE_AS2        117
#define NOTE_B2         123
#define NOTE_C3         131
#define NOTE_CS3        139
#define NOTE_D3         147
#define NOTE_DS3        156
#define NOTE_E3         165
#define NOTE_F3         175
#define NOTE_FS3        185
#define NOTE_G3         196
#define NOTE_GS3        208
#define NOTE_A3         220
#define NOTE_AS3        233
#define NOTE_B3         247
#define NOTE_C4         262
#define NOTE_CS4        277
#define NOTE_D4         294
#define NOTE_DS4        311
#define NOTE_E4         330
#define NOTE_F4         349
#define NOTE_FS4        370
#define NOTE_G4         392
#define NOTE_GS4        415
#define NOTE_A4         440
#define NOTE_AS4        466
#define NOTE_B4         494
#define NOTE_C5         523
#define NOTE_CS5        554
#define NOTE_D5         587
#define NOTE_DS5        622
#define NOTE_E5         659
#define NOTE_F5         698
#define NOTE_FS5        740
#define NOTE_G5         784
#define NOTE_GS5        831
#define NOTE_A5         880
#define NOTE_AS5        932
#define NOTE_B5         988
#define NOTE_C6         1047
#define NOTE_CS6        1109
#define NOTE_D6         1175
#define NOTE_DS6        1245
#define NOTE_E6         1319
#define NOTE_F6         1397
#define NOTE_FS6        1480
#define NOTE_G6         1568
#define NOTE_GS6        1661    
#define NOTE_A6         1760
#define NOTE_AS6        1865
#define NOTE_B6         1976
#define NOTE_C7         2093
#define NOTE_CS7        2217
#define NOTE_D7         2349
#define NOTE_DS7        2489
#define NOTE_E7         2637
#define NOTE_F7         2794
#define NOTE_FS7        2960
#define NOTE_G7         3136
#define NOTE_GS7        3322
#define NOTE_A7         3520
#define NOTE_AS7        3729
#define NOTE_B7         3951
#define NOTE_C8         4186
#define NOTE_CS8        4435
#define NOTE_D8         4699
#define NOTE_DS8        4978

/*********************  Define for IR remote control ***********************/
#define KEY_0				0xFF9867
#define KEY_1				0xFFA25D
#define KEY_2				0xFF629D
#define KEY_3				0xFFE21D
#define KEY_4				0xFF22DD
#define KEY_5				0xFF02FD
#define KEY_6				0xFFC23D
#define KEY_7				0xFFE01F
#define KEY_8				0xFFA857
#define KEY_9				0xFF906F
	
#define KEY_STAR			0xFF6897
#define KEY_PROM			0xFFB04F
	
#define KEY_UP				0xFF18E7	
#define KEY_DOWN			0xFF4AB5
#define KEY_LEFT			0xFF10EF
#define KEY_RIGHT			0xFF5AA5
	
#define KEY_OK				0xFF38C7
/*********************  Vn Board GPIO Map *********************************/


// struct defined in VnPort.h

VnPort_Sig vnPort[17] =
{
  { NC, NC }, { 17, 16 }, { NC, NC }, { NC, NC }, {  2,  12 },
  { 38, A0 }, { NC, NC }, { NC, 27 }, { NC, 32 }, {  NC,  25 },
  { NC, 39 }, { NC, NC }, { NC, NC }, { NC, NC }, { NC, NC },
  { NC, NC },{ NC, NC },
};

VnMotor	vnMotor[4] = 
{
	{NC, NC, NC}, {14, 15, 4}, {18, 19, 33}, {26, 27, 13},
};
/**
  *Port[1]: Ultrasonic sensor
  *      19: Triger pin
  *      18: Echo pin
  * DC motor control port
  *     M1: 10 - speed
  *         11 - dir
  *     M2: 12 - speed
  *         13 - dir
*/

// VnPort_Sig vnPort[17] =
// { 
 // { 22,23 }, { 18,19 }, { 22, 23 }, { 12, 27 }, {  8,  2 },
 // { NC, NC }, { 22, NC }, { 26, 27 }, { NC, NC}, {  26,  27 },
 // {  5,  4 }, { NC, NC }, { NC, NC }, { NC, NC }, { NC, NC },
 // { NC, NC },{ NC, NC },
// };
#endif