/* Global Config */
/*---------------------------------------------------------------------------*/
#if (1) 
#define ROBOX_LOG(...)   Serial.printf(__VA_ARGS__)
#else
#define ROBOX_LOG(...)
#endif

//#define MOTOR_TYPE_3
//#define VBOT_CHAIN


/* include */
/*---------------------------------------------------------------------------*/
#include "VbotBoard.h"
#include "BluetoothSerial.h"
#include "FastLED.h"

/* Global Structure */
/*---------------------------------------------------------------------------*/
union{
  byte byteVal[2];
  short shortVal;
}valShort;

/* Function Prototype */
/*---------------------------------------------------------------------------*/
void robotInit(void);
void robotStartup(void);
void BleInit(void);
static void serialHandle();
static void soundEffect(void);
static void robotFollowingLine(void);
static void go_in_circle(void);
static void go_demo_srf05_lighsensor(void);

/* Global Define */
/*---------------------------------------------------------------------------*/
#define GET 1
#define RUN 2
#define RESET 4
#define START 5

#define ULTRASONIC_SENSOR       1
#define LIGHT_SENSOR_VALUE      3
#define JOYSTICK                5
#define RGBLED                  8
#define TONE                    34
#define LINEFOLLOWER            17

#define LINE_DETECT_MODE      100
#define LINE_CIRCLE_MODE      101
#define SOUND_FOLLOW_MODE     102
#define NORMAL_MODE           103


#define NUM_LEDS 2
#define LED_DATA_PIN 23

#define DEVICE_NAME "Robox"
/* Hardware API */
/*---------------------------------------------------------------------------*/
VnDCMotor           DcMotorL(M2);
VnDCMotor           DcMotorR(M3);
CRGB                leds[NUM_LEDS];
VnUltrasonicSensor  Ultra(ULTRA);
VnBuzzer            Buzzer;
VnLineFollower      LINE(FLOWLINE);
VnLightSensor       LightSensor(LIGHT_SENSOR);
VnSoundSensor       SoundSensor(SOUND_SENSOR);
/* Global Variable */
/*---------------------------------------------------------------------------*/
BluetoothSerial SerialBT;
TaskHandle_t Task1;

/* Setup function */
/*---------------------------------------------------------------------------*/
void setup()
{
  robotInit();
	Serial.begin(115200);

  robotStartup();

	SerialBT.begin(DEVICE_NAME);
  BleInit();

  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1", /* Name of the task */
    8024,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */
}

/* Main Loop */
/*---------------------------------------------------------------------------*/
int i = 0;
void loop() 
{
  serialHandle();
}

int mode = NORMAL_MODE;
void Task1code( void * parameter) 
{
  while(1) {
    switch (mode)
    {
    case NORMAL_MODE:
      /* do nothing */
      break;
    case LINE_DETECT_MODE: {
        robotFollowingLine();
        //ROBOX_LOG("LINE_DETECT_MODE\n");
      }
      break;
    case LINE_CIRCLE_MODE: {
        go_in_circle();
        //ROBOX_LOG("LINE_CIRCLE_MODE\n");
      }
      break;
    case SOUND_FOLLOW_MODE: {
        soundEffect();
        //ROBOX_LOG("SOUND_FOLLOW_MODE\n");
      }
      break;
    default:
      break;
    }
    delay(100);
  }
}

void robotInit(void)
{
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;
  FastLED.show();  
}

void robotStartup(void) 
{
  for (i = 0; i < 5; i++) {
    robotSetLed(0, 255, 0, 0);
    delay(500);
    robotSetLed(0, 0, 0, 0);
    delay(500);
  }
}

void robotSetJoyStick(int leftSpeed, int rightSpeed)
{
  DcMotorL.run(leftSpeed, MOTOR2);
  DcMotorR.run(rightSpeed, MOTOR3);
  ROBOX_LOG("JOYSTICK leftSpeed=%d, rightSpeed=%d\n", leftSpeed, rightSpeed);
}

static int rgb2HexColor(int r, int g, int b)
{
    return (r<<16) | (g<<8) | b;
}

void robotSetLed(int idx, int r, int g, int b)
{
  if (idx < 0 || idx > NUM_LEDS)
    return;

  int color = rgb2HexColor(r, g, b);
  if (idx == 0) {
    for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = color;
  } else {
      leds[idx - 1] = color;
  }
  FastLED.show();
  
  ROBOX_LOG("LEd idx=%d, r=%d, g=%d, b=%d\n", idx, r, g, b);
}

void robotSetTone(int freq, int duration)
{
  Buzzer.tone(freq, duration);
  ROBOX_LOG("TONE freq=%d, duration=%d\n", freq, duration);
}

float robotGetDistance(void)
{
  float distance = (float)Ultra.distanceCm(200);
  ROBOX_LOG("DISTANCE distance=%.2f\n", distance);
  return distance;
}

float robotGetLineSensor(void)
{
  float line = (float)(LINE.readSensor1() * 2 + LINE.readSensor2());
  ROBOX_LOG("LINE LEFT=%d RIGHT=%d\n", LINE.readSensor1(), LINE.readSensor2());
  return line;
}

float robotGetLightSensor(void)
{
  float value = (float)(LightSensor.read() * 1000 / 4095);
  ROBOX_LOG("LIGHTSENSOR LEFT=%.1f   %d\n", value, LightSensor.read());
  return value;
}
/* Classic Bluetooth API for Android */
/*---------------------------------------------------------------------------*/
char buffer[64];
boolean isAvailable = false;
boolean isStart     = false;
char serialRead     = 0;
unsigned char prevc = 0;
byte index2          = 0;
byte dataLen        = 0;
uint8_t command_index = 0;
union
{
  byte byteVal[4];
  float floatVal;
  long longVal;
}val;

static void writeBuffer(int index1, unsigned char c){
  buffer[index1] = c;
}

static void readSerial(){
  isAvailable = false;
  if(SerialBT.available()>0){
    isAvailable = true;
    serialRead = SerialBT.read();
  }
}

static void serialHandle(){
  readSerial();
  if(isAvailable){
    unsigned char c = serialRead&0xff;
    if(c == 0x55 && isStart == false){
     if(prevc == 0xff){
      index2 = 1; 
      isStart = true;
     }
    }else{
      prevc = c;
      if (isStart){
        if (index2 == 2){
         dataLen = c; 
        }else if (index2 > 2){
          dataLen--;
        }
        writeBuffer(index2,c);
      }
    }
     index2++;
     if(index2>51){
      index2=0; 
      isStart=false;
     }
     if (isStart && dataLen == 0 && index2 > 3){ 
        isStart = false;
        parseData();
        index2 = 0;
     }
  }
}

void writeHead()
{
  writeSerial(0xff);
  writeSerial(0x55);
}

static void writeSerial(unsigned char c){
  SerialBT.write(c);
}

static void writeEnd(){
  SerialBT.println(); 
}

static void callOK(){
    writeSerial(0xff);
    writeSerial(0x55);
    writeEnd();
}

static unsigned char readBuffer(int index){
  return buffer[index]; 
}

static short readShort(int idx){
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}

static void runModule(int device){
  //0xff 0x55 0x6 0x0 0x2 0x22 0x9 0x0 0x0 0xa
  ROBOX_LOG("Received command\n");
  int port = readBuffer(6);
  int pin = port;
  switch (device) {
    case JOYSTICK:{
      int leftSpeed = readShort(6);
      int rightSpeed = readShort(8);
      robotSetJoyStick(leftSpeed, rightSpeed);
      break;
    }
    case RGBLED:{
      int idx = readBuffer(8);
      int r = readBuffer(9);
      int g = readBuffer(10);
      int b = readBuffer(11);
      robotSetLed(idx, r, g, b);
      break;
    }
    case TONE:{
      int freq = readShort(6);  
      int duration = readShort(8);
      robotSetTone(freq, duration);
      break;
    }
    case LINE_DETECT_MODE:
    case LINE_CIRCLE_MODE:
    case SOUND_FOLLOW_MODE:
    case NORMAL_MODE:
      ROBOX_LOG("Mode=%d\n", mode);
      mode = device;
      break;
  }
}

void sendFloat(float value)
{ 
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

void readSensor(int device)
{
  /**************************************************
      ff    55      len idx action device port slot data a
      0     1       2   3   4      5      6    7    8
      0xff  0x55   0x4 0x3 0x1    0x1    0x1  0xa 
  ***************************************************/
  float value=0.0;
  int port,slot,pin;
  port = readBuffer(6);
  pin = port;
  switch(device)
  {
    case ULTRASONIC_SENSOR:
    {
      value = (float)robotGetDistance();
      writeHead();
      writeSerial(command_index);
      sendFloat(value);
    }
    break;
    case LINEFOLLOWER:
    {
      value = robotGetLineSensor();
      sendFloat(value);
    }
    case LIGHT_SENSOR_VALUE:
    {
      value = robotGetLightSensor();
      sendFloat(value);
    }
  }
}

static void parseData(){
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  int action = readBuffer(4);
  int device = readBuffer(5);
  switch(action){
    case GET:{
      if(device != ULTRASONIC_SENSOR) {
        writeHead();
        writeSerial(idx);
      }
       readSensor(device);
       writeEnd();
     }
     break;
     case RUN:{
       runModule(device);
       //callOK();
     }
      break;
      case RESET:{
        //reset
        callOK();
      }
     break;
     case START:{
        //start
        callOK();
      }
     break;
  }
}

/* Bluetooth Low Energy API for iOs */
/*---------------------------------------------------------------------------*/
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define ROBOT_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class BleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      uint8_t address[6] = {0xe0, 0x99, 0x71, 0xb6, 0x02, 0xc4};
      ROBOX_LOG("BLE Connected\n");
    };

    void onDisconnect(BLEServer* pServer) {
      ROBOX_LOG("BLE Disconnected\n");
    }
};

class BLERobotCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t charBuf[64];      
      std::string value = pCharacteristic->getValue();

      ROBOX_LOG("BLE write: ");
      for (int i = 0; i < value.length(); i++) {
        charBuf[i] = value[i];
        ROBOX_LOG("%x ", charBuf[i]);
      }
      ROBOX_LOG("\n");
            
      if (value.length() >= 2) {
        if (value[0] == RUN) {
          if (value[1] == JOYSTICK) {
            joystick(charBuf + 2, value.length() - 2);
          } else if (value[1] == RGBLED) {
            ledRgb(charBuf + 2, value.length() - 2);
          } else if (value[1] == TONE) {
            buzzer(charBuf + 2, value.length() - 2);
          }
        }
      }
    }

    void onRead(BLECharacteristic *pCharacteristic) {
      uint8_t buf[5] = {1, 2, 3, 4, 5};
      ROBOX_LOG("reading...\n");
      pCharacteristic->setValue("anh yeu em");
    }
    
    void ledRgb(uint8_t *value, uint8_t len) {
      if (value && len == 4) {
        uint8_t idx = value[0];
        uint8_t r = value[1];
        uint8_t g = value[2];
        uint8_t b = value[3];
        robotSetLed(idx, r, g, b);
      }
    }

    void joystick(uint8_t *value, uint8_t len) {
          if (value && len == 4) {
            int16_t leftSpeed = (int16_t)(value[0] << 8) + value[1];
            int16_t rightSpeed = (int16_t)(value[2] << 8) + value[3];
            robotSetJoyStick(leftSpeed, rightSpeed);
          }
     }
    
    void buzzer(uint8_t *value, uint8_t len) {
          if (value && len == 4) {
            int16_t freq = (int16_t)(value[0] << 8) + value[1];
            int16_t duration = (int16_t)(value[2] << 8) + value[3];
            robotSetTone(freq, duration);
          }
     }
};


void BleInit(void)
{
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  BLEDevice::setMTU(512);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         ROBOT_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE);                                 
  pCharacteristic->setCallbacks(new BLERobotCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData adv1;
  adv1.setName(DEVICE_NAME);
  pAdvertising->setAdvertisementData(adv1);
  
  BLEAdvertisementData adv;
  adv.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setScanResponseData(adv);
  
  pAdvertising->start();
}
static void soundEffect(void)
{
  int sound_detect = 0;
  int state = 0;
  
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Blue;
  }
  FastLED.show();
  
  int time = millis();
  while ((millis() - time) < 2500) {
    sound_detect = SoundSensor.readSoundSignal();
    if (sound_detect == 0) {
      delay(100);
      state++;  
    } 
  }
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  switch (state) {
    case 1:
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(170, MOTOR3);
      delay(1000);
      break;
    case 2:
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(-170, MOTOR3);
      delay(1000);
      break;
    case 3:
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(-170, MOTOR3);
      delay(1000);
      break;
    case 4:  
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(170, MOTOR3);
      delay(1000);
      break;
    case 5:  
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::Green;
      }
      FastLED.show();
      delay(500);
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::Black;
      }
      FastLED.show();
      
      go_demo_srf05_lighsensor();
      delay(1000);
      break;
    default:
      delay(1000);
      break;  
  }
  DcMotorL.run(0, MOTOR2);
  DcMotorR.run(0, MOTOR3);
}
static void robotFollowingLine(void)
{
  if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 0)) {
      //Serial.println("find");
      DcMotorL.run(190, MOTOR2);
      DcMotorR.run(170, MOTOR3);
  }
  else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 0)) {
      //Serial.println("left");
      DcMotorL.run(-180, MOTOR2);
      DcMotorR.run(-160, MOTOR3);
  }
  else if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 1)) {
       //Serial.println("right");
      DcMotorL.run(190, MOTOR2);
      DcMotorR.run(160, MOTOR3);
  }
  else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 1)) {
      //Serial.println("forward");
      DcMotorL.run(-170, MOTOR2);
      DcMotorR.run(160, MOTOR3);
  }
}

static void go_in_circle(void)
{
    if ((LINE.readSensor1() == 0) && (LINE.readSensor2() == 0)) {
      DcMotorL.run(-160, MOTOR2);
      DcMotorR.run(160, MOTOR3);
    }
    else if ((LINE.readSensor1() == 1) && (LINE.readSensor2() == 1)) {
      Serial.println("forward");
      DcMotorL.run(160, MOTOR2);
      DcMotorR.run(-160, MOTOR3);
      delay(500);
      DcMotorL.run(170, MOTOR2);
      DcMotorR.run(190, MOTOR3);
      delay(1000);
    }
}
static void go_demo_srf05_lighsensor(void)
{
  while (SoundSensor.readSoundSignal()) {
    if (Ultra.distanceCm(200) <= 10) {
      Buzzer.tone(NOTE_A4, 700);
      Buzzer.tone(NOTE_F5, 700);
    }
    Serial.println(LightSensor.read());
    if (LightSensor.read() < 1000) {
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::White;
      }
      FastLED.show();
    }
    else{
      for (int i = 0; i < NUM_LEDS; i++){
        leds[i] = CRGB::Black;
      }
      FastLED.show();
    }
  }
}
