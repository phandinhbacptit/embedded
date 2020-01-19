#include "VbotBoard.h"
#include "BluetoothSerial.h"
#include "FastLED.h"

/* Global Config */
/*---------------------------------------------------------------------------*/
#if (1) 
#define ROBOX_LOG(...)   Serial.printf(__VA_ARGS__)
#else
#define ROBOX_LOG(...)
#endif

/* Global Structure */
/*---------------------------------------------------------------------------*/
union{
  byte byteVal[2];
  short shortVal;
}valShort;

/* Function Prototype */
/*---------------------------------------------------------------------------*/
void robotInit(void);
void BleInit(void);

/* Global Define */
/*---------------------------------------------------------------------------*/
#define GET 1
#define RUN 2
#define RESET 4
#define START 5

#define JOYSTICK    5
#define RGBLED      8
#define TONE        34

#define NUM_LEDS 2
#define LED_DATA_PIN 23

#define DEVICE_NAME "Robox"

/* Global Variable */
/*---------------------------------------------------------------------------*/
BluetoothSerial SerialBT;

/* Setup function */
/*---------------------------------------------------------------------------*/
void setup()
{
  robotInit();
	Serial.begin(115200);
	SerialBT.begin(DEVICE_NAME);
  BleInit();
}

/* Main Loop */
/*---------------------------------------------------------------------------*/
void loop() 
{
  serialHandle();
}

/* Hardware API */
/*---------------------------------------------------------------------------*/
VnDCMotor DcMotorL(M1);
VnDCMotor DcMotorR(M2);
CRGB leds[NUM_LEDS];

void robotInit(void)
{
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;
  FastLED.show();
}

void robotSetJoyStick(int leftSpeed, int rightSpeed)
{
  //DcMotorL.run(leftSpeed);
  //DcMotorR.run(rightSpeed);
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

  ROBOX_LOG("RGBLED idx=%d, r=%d, g=%d, b=%d\n", idx, r, g, b);
}

void robotSetTone(int freq, int duration)
{
  ROBOX_LOG("TONE freq=%d, duration=%d\n", freq, duration);
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
  }
}

static void parseData(){
  isStart = false;
  int idx = readBuffer(3);
  int action = readBuffer(4);
  int device = readBuffer(5);
  switch(action){
    case GET:{
//        writeHead();
//        writeSerial(idx);
//        readSensor(device);
//        writeEnd();
     }
     break;
     case RUN:{
       runModule(device);
       callOK();
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
