#include "VbotBoard.h"
//#include <FastLED.h>

using namespace std;

//#define TEST_GPIO  
//#define  TEST_DCMOTOR
//#define TEST_RGB
#define TEST_ULTRA
//#define TEST_BUZZER
//#define TEST_BLUETOOTH
//#define TEST_IR
//#define TEST_BUTTON
//#define TEST_FOLLOW_LINE
//#define TEST_LIGHT_SENSOR
//#define TEST_LED_MATRIX
//#define TEST_MPU6050

#ifdef VN_PORT_DEFINED
  
  #ifdef TEST_DCMOTOR
      VnDCMotor           DcMotor1(M1);
      VnDCMotor           DcMotor2(M2);
      VnDCMotor           DcMotor3(M3);
  #endif

  #ifdef TEST_GPIO
      VnPort              VnGPIO;
  #endif

  #ifdef TEST_ULTRA
      VnUltrasonicSensor  Ultra(ULTRA);
  #endif

  #ifdef TEST_BUZZER
      VnBuzzer            Buzzer;
  #endif

  #ifdef TEST_IR
      VnIrRemote          IR(IR_REMOTE);
      decode_results results;
  #endif

  #ifdef TEST_FOLLOW_LINE
        VnLineFollower  LINE(FLOWLINE);
  #endif
  
  #ifdef TEST_BUTTON
      VnButton            Btn(BUTTON);
  #endif
  
  #ifdef TEST_RGB
    VnRGB LED(RGB_LED);
  #endif
  
  #ifdef TEST_BLUETOOTH
    VnBle   bleDevice;
    extern bool deviceConnected;
    extern bool oldDeviceConnected;
    uint8_t txValue = 0;
  
    extern BLECharacteristic *VnBleTx;
    extern BLECharacteristic *VnBleRx;  
    extern BLEServer *VnServer;
    extern BLEService *VnService;
  #endif
  
  #ifdef TEST_LIGHT_SENSOR
    VnLightSensor LightSensor(LIGHT_SENSOR);
  #endif

  #ifdef TEST_LED_MATRIX
    Vn74hc595 Matrix(MAXTRIX);
  #endif

  #ifdef TEST_MPU6050
    VnMpu6050 MPU(MPU6050);
  #endif
 /***********Don't define VN_PORT_DEFINE********************/ 
#else
  #ifdef TEST_MOTOR
      VnDCMotor           DcMotor(22, 23, 1);
  #endif

  #ifdef TEST_ULTRA
      VnUltrasonicSensor Ultra(16,17);
  #endif
  
  #ifdef TEST_BUZZER
      VnBuzzer            Buzzer(22);
  #endif

  #ifdef TEST_IR
      VnIrRemote          IR(25);
  #endif
  
  #ifdef TEST_FOLLOW_LINE
      VnLineFollower  LINE(2, 12);
  #endif

  #ifdef TEST_BUTTON
      VnButton            Btn(39);
  #endif
  
  #ifdef TEST_RGB
    VnRGB LED(27);
  #endif
  
  #ifdef TEST_BLUETOOTH
    VnBle   bleDevice;
    extern bool deviceConnected;
    extern bool oldDeviceConnected;
    uint8_t txValue = 0;
  
    extern BLECharacteristic *VnBleTx;
    extern BLECharacteristic *VnBleRx;  
    extern BLEServer *VnServer;
    extern BLEService *VnService;
  #endif

  #ifdef TEST_LED_MATRIX
    Vn74hc595 Matrix(12, 2, 13);
  #endif
  
  #ifdef TEST_MPU6050
    VnMpu6050 MPU(21, 22);
  #endif
#endif


float i, j, f, k;
int channel = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
int melody[] = {
NOTE_G4,//5  
NOTE_G4,//5
NOTE_A4,//6
NOTE_G4,//5
NOTE_C5,//1.
NOTE_B4,//7
1,
NOTE_G4,//5
NOTE_G4,//5
NOTE_A4,//6
NOTE_G4,//5
NOTE_D5,//2.
NOTE_C5,//1.
1,
NOTE_G4,//5
NOTE_G4,//5
NOTE_G5,//5.
NOTE_E5,//3.
NOTE_C5,//1.
NOTE_B4,//7
NOTE_A4,//6
1,
NOTE_F5,//4.
NOTE_F5,//4.
NOTE_E5,//3.
NOTE_C5,//1.
NOTE_D5,//2.
NOTE_C5,//1.
1,
};

int noteDurations[] = {
  8,8,4,4,4,4,
  8,
  8,8,4,4,4,4,
  8,
  8,8,4,4,4,4,2,
  8,
  8,8,4,4,4,2,
  4,
};

std::string buffer;
int count = 0;
float offAccX = 0, offAccY = 0, offGyroX = 0, offGyroY = 0, offGyroZ = 0; 
void setup() 
{
    Serial.begin(115200);
    Serial.println("setup finish");
// put your setup code here, to run once:
#ifdef TEST_IR
  IR.enableIr();
#endif

#ifdef TEST_RGB
     LED.begin();
#endif 
#ifdef TEST_BLUETOOTH
    bleDevice.VnBleInit("PhanDinhBac");
#endif  

#ifdef TEST_MPU6050
    MPU.init(400000);
    MPU.sensorOffsetGet(&offAccX, &offAccY, &offGyroX, &offGyroY, &offGyroZ);
#endif

}

void loop() 
{
#ifdef TEST_BLUETOOTH  
    bleDevice.write("phandinhgiot");
    delay(1000);
    bleDevice.autoConnect();
#endif

#ifdef TEST_LIGHT_SENSOR
  Serial.print(LightSensor.read());
  Serial.print("   ");
  Serial.println(LightSensor.read() * 1000 / 4095);
  delay(100);
#endif

/*--------------Test GPIO----------------------*/
#ifdef TEST_GPIO
//     VnGPIO.dWrite1(HIGH);
//     VnGPIO.dWrite2(LOW);
//     delay(1000);
//     VnGPIO.dWrite1(LOW);
//     VnGPIO.dWrite2(HIGH);
//     delay(1000);

    for(i = 0; i< 255; i = i+ 5) {
        VnGPIO.aWrite1(i);
        delay(100);
    }
    for(i = 255; i> 0; i = i- 5) {
        VnGPIO.aWrite2(i);
        delay(100);
    }
    delay(1000); 
#endif
/*-----------------Test Button------------------*/
#ifdef TEST_BUTTON
    Btn.read_mode();
#endif
///*---------------Test DC motor----------------*/ 
#ifdef TEST_DCMOTOR
    DcMotor1.run(-75, MOTOR1);
    DcMotor2.run(-95, MOTOR2);
    DcMotor3.run(-5, MOTOR3);
    delay(2000);
    DcMotor1.run(75, MOTOR1);
    DcMotor2.run(75, MOTOR2);
    DcMotor3.run(75, MOTOR3);
    delay(2000);   
#endif
///*---------------Test led RGB-------------------- */
#ifdef TEST_RGB 
    color_loop();
#endif
/*---------------Test Ultra sonic-------------------- */
#ifdef TEST_ULTRA
    Serial.print("Distance:  ");
    Serial.println(Ultra.distanceCm(200));
#endif
///*---------------Test Buzzer-------------------- */
#ifdef TEST_BUZZER
    play();              //Play the music.
    delay(300);          //Pause for a while.
    Serial.print("+");
#endif
///*---------------Test Serial-------------------- */
//#ifdef TEST_SERIAL
//     Serial.println("Hello world............");
//     delay(1000); 
//#endif
//
//#ifdef TEST_IR
//  if (IR.decode(&results)) {
//      switch (results.value) {
//        case KEY_0:
//            Serial.println("Key_0");
//            break;
//        case KEY_1:
//            Serial.println("Key_1");
//            break;
//        case KEY_2:
//            Serial.println("Key_2");
//            break;
//        case KEY_3:
//            Serial.println("Key_3");
//            break;
//        case KEY_4:
//            Serial.println("Key_4");
//            break;
//        case KEY_5:
//            Serial.println("Key_5");
//            break;
//        case KEY_6:
//            Serial.println("Key_6");
//            break;
//        case KEY_7:
//            Serial.println("Key_7");
//            break;
//        case KEY_8:
//            Serial.println("Key_8");
//            break;
//        case KEY_9:
//            Serial.println("Key_9");
//            break;
//        case KEY_STAR:
//            Serial.println("Key_*");
//            break;
//        case KEY_PROM:
//            Serial.println("Key_#");
//            break;
//        case KEY_UP:
//            Serial.println("Key_up");
//            break;
//        case KEY_DOWN:
//            Serial.println("Key_down");
//            break;
//        case KEY_LEFT:
//            Serial.println("Key_left");
//            break;
//        case KEY_RIGHT:
//            Serial.println("Key_right");
//            break;
//        case KEY_OK:
//            Serial.println("Key_ok");
//            break;
//        default:
//            break;
//      }
//      IR.resume();  
//  }
//#endif

#ifdef TEST_FOLLOW_LINE
      Serial.print(LINE.readSensor1());
      Serial.print(" ");
      Serial.println(LINE.readSensor2());
#endif
#ifdef TEST_LED_MATRIX

      for (int cnt = 0; cnt < 1; cnt++) {
         Matrix.displayImage(face[cnt], 60);
      }

      for (int cnt = 0; cnt < 9; cnt++) {
         Matrix.displayImage(SHARP[cnt], 60);
      }
      
      for (int cnt = 0; cnt < 10; cnt++) {
         Matrix.displayImage(motion_effect1[cnt], 20);
      }

      for (int cnt = 0; cnt < 23; cnt++) {
         Matrix.displayImage(motion_effect2[cnt], 20);
      }

      for (int cnt = 0; cnt < 27; cnt++) {
         Matrix.displayImage(motion_effect3[cnt], 20);
      }
      
      for (int cnt = 0; cnt < 39; cnt++) {
         Matrix.displayImage(characterHEX[cnt], 100);
      }

//      Matrix.displayImage(all_off_effect, 40);
#endif
#ifdef TEST_MPU6050
    MPU.gyroRateGet(&gyroX, &gyroY, &gyroZ, offGyroX, offGyroY, offGyroZ);
    Serial.print(gyroX);
    Serial.print("\t");
    Serial.print(gyroY);
    Serial.print("\t");
    Serial.println(gyroZ);
#endif

}
#ifdef TEST_RGB
void color_loop()
{
    LED.clear();
    LED.setPixelColor(0, YELLOW); 
    LED.setPixelColor(1, LED.Color(0, 0, 255));
    LED.setBrightness(20);
    LED.show();
    delay(500);
    for (i = 0; i < 2; i++) {
      LED.setPixelColor(i, LED.Color(0, 0, 0));
      LED.show();
    }
    delay(500);
   
}

#endif

#ifdef TEST_BUZZER
void play()
{
    for (int thisNote = 0; thisNote < 29; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];
    Buzzer.tone(melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 0.1;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    Buzzer.noTone();
  }
}
#endif
