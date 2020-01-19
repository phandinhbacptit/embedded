#include "VbotBoard.h"                                          
/***********Define Ation**************************/
#define GET                    1
#define RUN                    2
#define RESET                  4
#define START                  5  
/***********Define Device*************************/
#define VERSION                0
#define ULTRASONIC_SENSOR      1
#define TEMPERATURE_SENSOR     2
#define LIGHT_SENSOR           3
#define POTENTIONMETER         4
#define JOYSTICK               5
#define GYRO                   6
#define SOUND_SENSOR           7
#define RGBLED                 8
#define SEVSEG                 9
#define MOTOR                  10
#define SERVO                  11
#define ENCODER                12
#define IR                     13
#define IRREMOTE               14
#define PIRMOTION              15
#define INFRARED               16
#define LINEFOLLOWER           17
#define IRREMOTECODE           18
#define SHUTTER                20
#define LIMITSWITCH            21
#define BUTTON                 22
#define HUMITURE               23
#define FLAMESENSOR            24
#define GASSENSOR              25
#define COMPASS                26
#define TEMPERATURE_SENSOR_1   27
#define DIGITAL                30
#define ANALOG                 31
#define PWM                    32
#define SERVO_PIN              33
#define TONE                   34
#define BUTTON_INNER           35
#define ULTRASONIC_ARDUINO     36
#define PULSEIN                37
#define TIMER                  50
/**********Define frequency for Buzzer*****************/
#define NTD1     294
#define NTD2     330
#define NTD3     350
#define NTD4     393
#define NTD5     441
#define NTD6     495
#define NTD7     556
#define NTDL1    147
#define NTDL2    165
#define NTDL3    175
#define NTDL4    196
#define NTDL5    221
#define NTDL6    248
#define NTDL7    278
#define NTDH1    589
#define NTDH2    661
#define NTDH3    700
#define NTDH4    786
#define NTDH5    882
#define NTDH6    990
#define NTDH7    112
/********Define Control mode **************/
#define BLUE_TOOTH             0
#define IR_CONTROLER           1
/********Define Type run of Vbot***********/
#define RUN_F    0x01
#define RUN_B    0x01<<1
#define RUN_L    0x01<<2
#define RUN_R    0x01<<3
#define STOP     0
/************Declare Analog pinout*********/
#define A0                     36
#define A3                     39
#define A4                     32
#define A5                     33
#define A6                     34
#define A7                     35
int analogs1[6]={A0, A3, A4, A5, A6, A7};
//int analogs2[6]={A0, A1, A5, A6, A7, A8, A9};
/************Define type varialble********************/
enum {
  MODE_CONTROL,
  MODE_AUTO_ULTR,
  MODE_AUTO_LINE
};

union {
  byte byteVal[4];
  float floatVal;
  long longVal;
} val;

union {
  byte byteVal[8];
  double doubleVal;
} valDouble;

union {
  byte byteVal[2];
  short shortVal;
} valShort;

typedef struct VnModule {
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} VnModule;
/************Declare parameter*************/
boolean isAvailable = false;
boolean isStart = false;
boolean buttonPressed = false;
boolean currentPressed = false;
boolean pre_buttonPressed = false;

char buffer[52];
char bufferBt[52];
char serialRead;
byte indexVbot = 0;
byte dataLen;
byte modulesLen = 0;
unsigned char prevc = 0;
uint8_t prevState = 0;


uint8_t high = 15;
uint8_t low  = 1;
int moveSpeed = 200;
int LineFollowFlag = 0;

uint8_t command_index = 0;
uint8_t controlflag = IR_CONTROLER;
uint8_t motor_sta = STOP;
uint8_t mode = MODE_CONTROL;

float angleServo = 90.0;
double lastTime = 0.0;
double currentTime = 0.0;
int   i = 0;

String vbotVersion = "24.10.018";

VnUltrasonicSensor       Ultra(ULTRA);
VnDCMotor                DcMotorL(M1);
VnDCMotor                DcMotorR(M2);
VnBuzzer                 Buzzer;
VnPort                   VnGPIO;
VnLineFollower           Line(FLOWLINE);
//VnLightSensor            Light(LIGHT);
VnServo                  vServo(SERVO);

void setup() {
  // put your setup code here, to run once:
//  Buzzer.tone(500,50); 
//  delay(50);
//  buzzerOff();
  Serial.print("Version: ");
  Serial.println(vbotVersion);
//  vServo.attach(1);
}

void loop() {
//  VnServo                  vServo(SERVO);
  Serial.printf("\n ++");
  delay(1000);
//  for (i = 0; i < 8888; i ++) {
//  vServo.write(i);
//   delay(80);
//  }
//  while(1) {
//    get_ir_command();
//    serialHandle();
//    currentPressed = !(analogRead(7) > 100);
//    if (currentPressed != pre_buttonPressed) {
//      //if((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//      //  rgb.reset(PORT_7,SLOT2);;
//      //}
//      pre_buttonPressed = currentPressed;
//      if (currentPressed == true)
//      {
//        if (mode == MODE_CONTROL) {
//          mode = MODE_AUTO_ULTR;
//          moveSpeed = 200;
//          Stop();
//          cli();
//          Buzzer.tone(NTD2, 300);
//          sei();
//          Buzzer.noTone();
//          //rgb.setColor(0,0,0);
//          //rgb.setColor(0, 10, 0);
//          //rgb.show();
//        }
//        else if (mode == MODE_AUTO_ULTR) {
//          mode = MODE_AUTO_LINE;
//          moveSpeed = 200;
//          Stop();
//          cli();                            // No interrupt
//          Buzzer.tone(NTD2, 300);
//          sei();                            // Interrupt
//          Buzzer.noTone();
//          //rgb.setColor(0,0,0);
//          //rgb.setColor(0, 0, 10);
//          //rgb.show();
//        }
//        else if (mode == MODE_AUTO_LINE){
//          mode = MODE_CONTROL;
//          moveSpeed = 220;
//          Stop();
//          cli();
//          Buzzer.tone(NTD1, 300);
//          sei();
//          Buzzer.noTone();
//          //rgb.setColor(0,0,0);
//          //rgb.setColor(10, 10, 10);
//          //rgb.show();
//        }
//      }
//    }
//    switch (mode) {
//      case MODE_CONTROL:
//        modeControl();
//        break;
//      case MODE_AUTO_ULTR:
//        modeAutoUltr();
//        break;
//      case MODE_AUTO_LINE:
//        modeAutoLine();
//        break;
//    }
//  }
}

/**
  * @Brief :  Read the index data in the bufer
  * @Param :  index - the position in buffer want to read
  * @Retval:  None
  */
unsigned char readBuffer(int16_t index)
{
  return buffer[index]; 
}
/**
  * @Brief :  Save data to buffer
  * @Param :  index - the position in buffer want to write
  *           c     - Data write to buffer
  * @Retval:  None
  */
void writeBuffer(int16_t index, unsigned char c)
{
  buffer[index]=c;
}
/**
  * @Brief :  Write header
  * @Param :  None
  * @Retval:  None
  */
void writeHead()
{
  writeSerial(0xff);
  writeSerial(0x55);
}
/**
  * @Brief :  Write character down one line
  * @Param :  None
  * @Retval:  None
  */
void writeEnd()
{
  Serial.println(); 
}
/**
  * @Brief :  Write 1 bytes through serial
  * @Param :  c - Data will be write
  * @Retval:  None
  */
void writeSerial(unsigned char c)
{
  Serial.write(c);
}
/**
  * @Brief :  Read data from serial 
  * @Param :  None
  * @Retval:  None
  */
void readSerial()
{
  isAvailable = false;
  if(Serial.available() > 0) {
    isAvailable = true;
    serialRead = Serial.read();
  }
}
/**
  * @Brief :  Write header 0xff and 0x55
  * @Param :  None
  * @Retval:  None
  */
void callOK()
{
  writeSerial(0xff);        
  writeSerial(0x55);
  writeEnd();
}
/**
  * @Brief :  Handle serial data
  * @Param :  None
  * @Retval:  None
  */
void serialHandle()
{
  readSerial();
  if (isAvailable) {
    unsigned char c = serialRead & 0xff;
    if ((c == 0x55) && (isStart == false)) {
      if (prevc == 0xff) {                               // Check if receive header 0xff 0x55 will set isStart = true
          indexVbot = 1;
          isStart = true;                                // Start save data received to buffer
      }
    }
    else {
      prevc = c;
      if (isStart) {
        if (indexVbot == 2) {                            // Get length from serial buffer 
          dataLen = c; 
        }
        else if (indexVbot > 2) {                
          dataLen--;                                     // After receive 1 byte decrease 1 unit
        }
        writeBuffer(indexVbot,c);                        // Save data received in the buffer with index
      }
    }
    indexVbot++;
    if (indexVbot > 51) {
      indexVbot=0; 
      isStart = false;
    }
    if (isStart && (dataLen == 0) && (indexVbot > 3)) { // Receive all frame from PC/Smartphone
      isStart = false;                                  // Stop received data, go parsing data
      parseData(); 
      indexVbot = 0;
    }
  }
}
/**
  * @Brief :  Parsing data. Depend on the action get will do accordingly request.
  * @Param :  value - the value to be transmiter
  * @Retval:  None
  * @Note  :
      ff    55     len idx action device port slot data a
      0     1       2   3    4      5      6    7    8
      0xff  0x55   0x4 0x3  0x1    0x1    0x1  0xa 
  */
void parseData()
{
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  int action = readBuffer(4);    // Reference define Action above
  int device = readBuffer(5);    // Reference define Device above
  switch (action) {
    case GET: 
    {
      if (device != ULTRASONIC_SENSOR) {
        writeHead();
        writeSerial(idx);
      }
      readSensor(device);        // Read data from peripheral          
      writeEnd();
    }
    break;
    case RUN:
    {
      runModule(device);        // Run action
      callOK();                  // Write header through serial
    }
    break;
    case RESET:
    {
      //reset
      DcMotorL.run(0);
      DcMotorR.run(0);
      buzzerOff();
      callOK();
    }
    break;
    case START:
    {
      //start
      callOK();
    }
    break;
  }
}

void get_ir_command()
{
//  static long time = millis();
//  if (ir.decode()) {
//    uint32_t value = ir.value;
//    time = millis();
//    switch ((value >> 16) & 0xff)
//    {
//      case IR_BUTTON_A:
//        controlflag = IR_CONTROLER;
//        moveSpeed = 220;
//        mode = MODE_CONTROL;
//        Stop();
//        cli();
//        buzzer.tone(NTD1,300);
//        sei();
//        if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//          rgb.reset(PORT_7,SLOT2);;
//        }
//        rgb.setColor(0,0,0);
//        rgb.setColor(10,10,10);
//        rgb.show();
//        break;
//      case IR_BUTTON_B:
//        controlflag = IR_CONTROLER;
//        moveSpeed = 200;
//        mode = MODE_AUTO_ULTR;
//        Stop();
//        cli();
//        buzzer.tone(NTD2,300);
//        sei();
//        if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//          rgb.reset(PORT_7,SLOT2);;
//        }
//        rgb.setColor(0,0,0);
//        rgb.setColor(0,10,0);
//        rgb.show();
//        break;
//      case IR_BUTTON_C:
//        controlflag = IR_CONTROLER;
//        moveSpeed = 200;
//        mode = MODE_AUTO_LINE;
//        Stop();
//        cli();
//        buzzer.tone(NTD3,300);
//        sei();
//        if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//          rgb.reset(PORT_7,SLOT2);;
//        }
//        rgb.setColor(0,0,0);
//        rgb.setColor(0,0,10);
//        rgb.show();
//        break;
//      case IR_BUTTON_PLUS:
//        controlflag = IR_CONTROLER;
//        if (mode == MODE_CONTROL) {
//          motor_sta = RUN_F;
//          if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//            rgb.reset(PORT_7,SLOT2);;
//          }
//          rgb.setColor(0,0,0);
//          rgb.setColor(10,10,0);
//          rgb.show();
//        }
//        break;
//      case IR_BUTTON_MINUS:
//        controlflag = IR_CONTROLER;
//        if (mode == MODE_CONTROL) {
//          motor_sta = RUN_B;
//          if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//            rgb.reset(PORT_7,SLOT2);;
//          }
//          rgb.setColor(0,0,0);
//          rgb.setColor(10,0,0);
//          rgb.show();
//        }
//        break;
//      case IR_BUTTON_NEXT:
//        controlflag = IR_CONTROLER;
//        if (mode == MODE_CONTROL) {
//          motor_sta = RUN_R;
//          if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//            rgb.reset(PORT_7,SLOT2);;
//          }
//          rgb.setColor(0,0,0);
//          rgb.setColor(1,10,10,0);
//          rgb.show();
//        }
//        break;
//      case IR_BUTTON_PREVIOUS:
//        controlflag = IR_CONTROLER;
//        if (mode == MODE_CONTROL) {
//          motor_sta = RUN_L;
//          if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//            rgb.reset(PORT_7,SLOT2);;
//          }
//          rgb.setColor(0,0,0);
//          rgb.setColor(2,10,10,0);
//          rgb.show();
//        }
//        break;
//      case IR_BUTTON_9:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTDH2, 300);
//        sei();
//        ChangeSpeed(factor * 9 + minSpeed);
//        break;
//      case IR_BUTTON_8:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTDH1, 300);
//        sei();
//        ChangeSpeed(factor * 9 + minSpeed);
//        break;
//      case IR_BUTTON_7:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTD7, 300);
//        sei();
//        ChangeSpeed(factor * 9 + minSpeed);
//        break;
//      case IR_BUTTON_6:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTD6, 300);
//        sei();
//        ChangeSpeed(factor * 6 + minSpeed);
//        break;
//      case IR_BUTTON_5:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTD5, 300);
//        sei();
//        ChangeSpeed(factor * 6 + minSpeed);
//        break;
//      case IR_BUTTON_4:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTD4, 300);
//        sei();
//        ChangeSpeed(factor * 6 + minSpeed);
//        break;
//      case IR_BUTTON_3:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTD3, 300);
//        sei();
//        ChangeSpeed(factor * 3 + minSpeed);
//        break;
//      case IR_BUTTON_2:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTD2, 300);
//        sei();
//        ChangeSpeed(factor * 3 + minSpeed);
//        break;
//      case IR_BUTTON_1:
//        controlflag = IR_CONTROLER;
//        cli();
//        buzzer.tone(NTD1, 300);
//        sei();
//        ChangeSpeed(factor * 3 + minSpeed);
//        break;
//    }
//  }
//  else if ((controlflag == IR_CONTROLER) && ((millis() - time) > 120)) {
//    motor_sta = STOP;
//    time = millis();
//    if (mode == MODE_CONTROL) {
//      if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//        rgb.reset(PORT_7,SLOT2);;
//      }
//      rgb.setColor(10, 10, 10);
//      rgb.show();
//    }
//    else if (mode == MODE_AUTO_ULTR) {
//      if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//        rgb.reset(PORT_7,SLOT2);;
//      }
//      rgb.setColor(0, 10, 0);
//      rgb.show();
//    }
//    else if (mode == MODE_AUTO_LINE) {
//      if ((rgb.getPort() != PORT_7) || rgb.getSlot() != SLOT2) {
//        rgb.reset(PORT_7,SLOT2);;
//      }
//      rgb.setColor(0, 0, 10);
//      rgb.show();
//    }
//  }
}
/**
  * @Brief :  Collect data from:
  *           - Sensor: Ultrasonic, temperature, light sensor, Pir, Line sensor,.... 
  *           - Peripheral: Button, ADC, DAC, GPIO, Timer, ...
  *           - Information: Vesion, ...
  * @Param :  device - device want to read data
  * @Retval:  None
  * @Note  :
      ff    55     len idx action device port slot data a
      0     1       2   3    4      5      6    7    8
      0xff  0x55   0x4 0x3  0x1    0x1    0x1  0xa 
  */
void readSensor(int device)
{ 
  float value = 0.0;
  int port, slot, pin;
  port = readBuffer(6);
  pin = port;
  switch(device)
  {
    case ULTRASONIC_SENSOR:
    {
      if (Ultra.getPort() != port) {        // Check if current port using with usltrasonic difference with port request from PC/smartphone
        Ultra.reset(port);                  // Resetup port
      }
      value = (float)Ultra.distanceCm(); 
      writeHead();                          // Write header 0xff, 0x55 prepare send data
      writeSerial(command_index);           // Command_index = idx
      sendFloat(value);                     // Send float value through serial
    }
    break;
    case TEMPERATURE_SENSOR:
    {
//      slot = readBuffer(7);
//      if((ts.getPort() != port) || (ts.getSlot() != slot))
//      {
//        ts.reset(port,slot);
//      }
//      value = ts.temperature();
//      sendFloat(value);
    }
    break;
    case LIGHT_SENSOR:
    case SOUND_SENSOR:
    case POTENTIONMETER:
    {
//      if(generalDevice.getPort() != port)
//      {
//        generalDevice.reset(port);
//        pinMode(generalDevice.pin2(),INPUT);
//      }
//      value = generalDevice.aRead2();
//      sendFloat(value);
    }
    break;
    case JOYSTICK:
    {
//      slot = readBuffer(7);
//      if(joystick.getPort() != port)
//      {
//        joystick.reset(port);
//      }
//      value = joystick.read(slot);
//      sendFloat(value); 
    }
    break;
    case IR:
    {
//      if(ir.getPort() != port)
//      {
//        ir.reset(port);
//      }
//      if(irReady)
//      {
//        sendString(irBuffer);
//        irReady = false;
//        irBuffer = "";
//      }
    }
    break;
    case PIRMOTION:
//    {
//      if(generalDevice.getPort() != port)
//      {
//        generalDevice.reset(port);
//        pinMode(generalDevice.pin2(),INPUT);
//      }
//      value = generalDevice.dRead2();
//      sendFloat(value);
//    }
    break;
    case LINEFOLLOWER:
//    {
//      if(generalDevice.getPort() != port)
//      {
//        generalDevice.reset(port);
//        pinMode(generalDevice.pin1(),INPUT);
//        pinMode(generalDevice.pin2(),INPUT);
//      }
//      value = generalDevice.dRead1()*2 + generalDevice.dRead2();
//      sendFloat(value);
//    }
    break;
    case BUTTON_INNER:
    {
      pin = analogs1[pin];
      char s = readBuffer(7);
      pinMode(pin,INPUT);
      boolean currentPressed = !(analogRead(pin)>10);
      sendByte(s^(currentPressed?1:0));
      buttonPressed = currentPressed;
    }
    break;
    case VERSION:
    {
      sendString(vbotVersion);
    }
    break;
    case DIGITAL:
    {
      pinMode(pin,INPUT);
      sendFloat(digitalRead(pin));      // Send state of Digital Pin to through Serial
    }
    break;
    case ANALOG:
    {
      pin = analogs1[pin];
      pinMode(pin,INPUT);
      sendFloat(analogRead(pin));         // Send analog value through Serial
    }
    break;
    case TIMER:
    {
      sendFloat(currentTime);             // Send time to PC/Smartphone
    }
    break;
  }
}
/**
  * @Brief :  Collect data from:
  *           - Sensor: Ultrasonic, temperature, light sensor, Pir, Line sensor,.... 
  *           - Peripheral: Button, ADC, DAC, GPIO, Timer, ...
  *           - Information: Vesion, ...
  * @Param :  device - device want to read data
  * @Retval:  None
  * @Note  :
      ff    55     len idx action device port slot data a
      0     1       2   3    4      5      6    7    8
      0xff  0x55   0x4 0x3  0x1    0x1    0x1  0xa 
 */
void runModule(int device)
{
  //0xff 0x55 0x6 0x0 0x2 0x22 0x9 0x0 0x0 0xa 
  int port = readBuffer(6);                               //read port
  int pin = port;
  switch(device)
  {
    case MOTOR:
    {
      controlflag = BLUE_TOOTH;                           // Control Vbot through bluetooth
      int16_t speed = readShort(7);                       // Read 2 bytes from 7 index in buffer read from serial
      port == M1?DcMotorL.run(speed):DcMotorR.run(speed); // Depend on value of M1 for run DcDcMotorL or DcDcMotorR.
    } 
    break;
    case RGBLED:
//    {
//      controlflag = BLUE_TOOTH;
//      int16_t slot = readBuffer(7);
//      int16_t idx = readBuffer(8);
//      int16_t r = readBuffer(9);
//      int16_t g = readBuffer(10);
//      int16_t b = readBuffer(11);
//
//      if((rgb.getPort() != port) || rgb.getSlot() != slot)
//      {
//        rgb.reset(port,slot);
//      }
//      if(idx > 0)
//      {
//        rgb.setColorAt(idx-1,r,g,b); 
//      }
//      else
//      {
//        rgb.setColor(r,g,b); 
//      }
//      rgb.show();
//    }
    break;
    case SERVO:
//    {
//      int16_t slot = readBuffer(7);
//      pin = slot == 1?mePort[port].s1:mePort[port].s2;
//      int16_t v = readBuffer(8);
//      if((v >= 0) && (v <= 180))
//      {
//        servo.attach(pin);
//        servo.write(v);
//      }
//    }
//    break;
//    case SEVSEG:
//    {
//      if(seg.getPort() != port)
//      {
//        seg.reset(port);
//      }
//      float v = readFloat(7);
//      seg.display(v);
//    }
    break;
    case LIGHT_SENSOR:
//    {
//      if(generalDevice.getPort() != port)
//      {
//        generalDevice.reset(port);
//      }
//      int16_t v = readBuffer(7);
//      generalDevice.dWrite1(v);
//    }
    break;
    case DIGITAL:
    {
      pinMode(pin,OUTPUT);
      int v = readBuffer(7);            // Write pin state
      digitalWrite(pin,v);
    }
    break;
    case PWM:
    {
      pinMode(pin,OUTPUT);
      int v = readBuffer(7);            // Write PWM value to pin
      ledcWrite(pin,v);
    }
    break;
    case TONE:
    {
      int hz = readShort(6);            // Read 2 bytes in buffer from index 6 to get frequency
      int tone_time = readShort(8);     // Get duration to make buzzer
      if (hz > 0) {
        Buzzer.tone(hz,tone_time);
      }
      else {
        Buzzer.noTone(); 
      }
    }
    break;
    case SERVO_PIN:
//    {
//      int v = readBuffer(7);
//      if((v >= 0) && (v <= 180))
//      {
//        servo.attach(pin);
//        servo.write(v);
//      }
//    }
    break;
    case TIMER:
    {
      lastTime = millis()/1000.0; 
    }
    break;
  }
}
/**
  * @Brief :  Turn on buzzer
  * @Param :  None
  * @Retval:  None
  */
void buzzerOn()
{
  Buzzer.tone(500,1000); 
}
/**
  * @Brief :  Turn off buzzer
  * @Param :  None
  * @Retval:  None
  */
void buzzerOff()
{
  Buzzer.noTone(); 
}
/*Regulation frame send data
*1 - Send Bytes
*2 - Send Float
*3 - Send Short
*4 - Send Len + String
*5 - Send Double
*/
/**
  * @Brief :  Send 1 byte through serial. Send 1 number after header
  * @Param :  c - the value to be transmiter
  * @Retval:  None
  */
void sendByte(char c)
{
  writeSerial(1);
  writeSerial(c);
}
/**
  * @Brief :  Send float value through serial. Send 2 number after header
  * @Param :  value - the value to be transmiter
  * @Retval:  None
  */
void sendFloat(float value)
{ 
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}
/**
  * @Brief :  Send 4 bytes through serial. Send 3 number after header
  * @Param :  value - the value to be transmiter
  * @Retval:  None
  */
void sendShort(double value)
{
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
  writeSerial(valShort.byteVal[2]);
  writeSerial(valShort.byteVal[3]);
}
/**
  * @Brief :  Send string through serial. Send 4 number after header
  * @Param :  value - the value to be transmiter
  * @Retval:  None
  */
void sendString(String s)
{
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for(int i=0;i<l;i++)
  {
    writeSerial(s.charAt(i));
  }
}
/**
  * @Brief :  Send 8 bytes through serial. Send 5 number after header
  * @Param :  value - the value to be transmiter
  * @Retval:  None
  */
void sendDouble(double value)
{
  writeSerial(5);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
  writeSerial(valDouble.byteVal[4]);
  writeSerial(valDouble.byteVal[5]);
  writeSerial(valDouble.byteVal[6]);
  writeSerial(valDouble.byteVal[7]);
}
/**
  * @Brief :  Read 2 bytes start at idx number
  * @Param :  idx - index to start read bytes
  * @Retval:  Short value
  */
short readShort(int idx)
{
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}
/**
  * @Brief :  Read 4 bytes start at idx number
  * @Param :  idx - index to start read bytes
  * @Retval:  float value
  */
float readFloat(int idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.floatVal;
}
/**
  * @Brief :  Run Vbot forward
  * @Param :  None
  * @Retval:  None
  */
void Forward()
{
  DcMotorL.run(-moveSpeed);
  DcMotorR.run(moveSpeed);
}
/**
  * @Brief :  Run Vbot Backward
  * @Param :  None
  * @Retval:  None
  */
void Backward()
{
  DcMotorL.run(moveSpeed); 
  DcMotorR.run(-moveSpeed);
}
/**
  * @Brief :  Run Vbot turn left
  * @Param :  None
  * @Retval:  None
  */
void TurnLeft()
{
  DcMotorL.run(moveSpeed*0.8);
  DcMotorR.run(moveSpeed*0.8);
}
/**
  * @Brief :  Run Vbot turn right
  * @Param :  None
  * @Retval:  None
  */
void TurnRight()
{
  DcMotorL.run(-moveSpeed*0.8);
  DcMotorR.run(-moveSpeed*0.8);
}
/**
  * @Brief :  Run Vbot turn left
  * @Param :  None
  * @Retval:  None
  */
void TurnLeft2()
{
  DcMotorL.run(-moveSpeed/5);
  DcMotorR.run(moveSpeed);
}
/**
  * @Brief :  Run Vbot turn right
  * @Param :  None
  * @Retval:  None
  */
void TurnRight2()
{
  DcMotorL.run(-moveSpeed);
  DcMotorR.run(moveSpeed/5);
}
/**
  * @Brief :  Run Vbot Backward and TurnLeft
  * @Param :  None
  * @Retval:  None
  */
void BackwardAndTurnLeft()
{
  DcMotorL.run(moveSpeed/3 ); 
  DcMotorR.run(-moveSpeed);
}
/**
  * @Brief :  Run Vbot Backward and TurnRight
  * @Param :  None
  * @Retval:  None
  */
void BackwardAndTurnRight()
{
  DcMotorL.run(moveSpeed); 
  DcMotorR.run(-moveSpeed/3);
}
/**
  * @Brief :  Stop Vbot
  * @Param :  None
  * @Retval:  None
  */
void Stop()
{
//  rgb.setColor(0,0,0);
//  rgb.show();
  DcMotorL.run(0);
  DcMotorR.run(0);
}
/**
  * @Brief :  Change speed of Vbot  
  * @Param :  spd - value of speed want to change.
  * @Retval:  None
  */
void ChangeSpeed(int spd)
{
  moveSpeed = spd;
}
/**
  * @Brief :  Robot move following signal control from IR remote 
  * @Param :  None
  * @Retval:  None
  */
void modeControl()
{
  switch (motor_sta)
  {
    case RUN_F:
      Forward();
      prevState = motor_sta;
      break;
    case RUN_B:
      Backward();
      prevState = motor_sta;
      break;
    case RUN_L:
      TurnLeft();
      prevState = motor_sta;
      break;
    case RUN_R:
      TurnRight();
      prevState = motor_sta;
      break;
    case STOP:
      if (prevState != motor_sta) {
        prevState = motor_sta;
        Stop();
      }
      break;
  }
}
/**
  * @Brief :  Vbot move following distance between Vbot and Object. When distance < lowvalue, 
  *           Vbot can turn left or right depend on the random value.
  * @Param :  None
  * @Retval:  None
  */
void modeAutoUltr()
{
  uint8_t d = Ultra.distanceCm(70);
  static long time = millis();
  randomSeed(analogRead(6));
  uint8_t randNumber = random(2);
  if ((d > high) || (d == 0)) {         // Vbot run forward if distance > high value.
    Forward();
  }
  else if ((d > low) && (d < high)) {   // Vbot random turn left/right depend on random value if distance < high value.
    switch (randNumber) {
      case 0:
        TurnLeft();
        delay(300);
        break;
      case 1:
        TurnRight();
        delay(300);
        break;
    }
  }
  else {
    switch (randNumber) {
      case 0:
        TurnLeft();
        delay(800);
        break;
      case 1:
        TurnRight();
        delay(800);
        break;
    }
  }
  delay(100);
}

void modeAutoLine()
{
  uint8_t val;
  val = Line.readSensors();   // Get state of 2 sensor line. State can be 00 01 10 11
  if (moveSpeed > 230) {
    moveSpeed = 230;
  }
  switch(val)
  {
    case S1_IN_S2_IN:
      Forward();
      LineFollowFlag = 10;
      break;

    case S1_IN_S2_OUT:
       Forward();
      if (LineFollowFlag > 1) {
        LineFollowFlag--;
      }
      break;

    case S1_OUT_S2_IN:
      Forward();
      if (LineFollowFlag < 20) {
        LineFollowFlag++;
      }
      break;

    case S1_OUT_S2_OUT:
      if (LineFollowFlag == 10) {
        Backward();
      }
      if (LineFollowFlag < 10) {
        TurnLeft2();
      }
      if (LineFollowFlag > 10) {
        TurnRight2();
      }
      break;
  }
}
