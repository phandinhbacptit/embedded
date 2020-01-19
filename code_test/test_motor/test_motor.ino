#include "VbotBoard.h"

using namespace std;

VnDCMotor           DcMotor2(M2);
VnDCMotor           DcMotor3(M3);
VnIrRemote          IR(IR_REMOTE);
decode_results results;

void setup() 
{
    Serial.begin(115200);
    Serial.println("setup finish");
    IR.enableIr();
}

void loop() 
{
  if (IR.decode(&results)) {
      switch (results.value) {
        case KEY_UP:
            Serial.println(" Tien");
            DcMotor2.run(95, MOTOR2);
            DcMotor3.run(95, MOTOR3);
            break;
        case KEY_DOWN:
            Serial.println(" Lùi");
            DcMotor2.run(-95, MOTOR2);
            DcMotor3.run(-95, MOTOR3);
            break;
        case KEY_LEFT:
            Serial.println(" trai");
            DcMotor2.run(-95, MOTOR2);
            DcMotor3.run(95, MOTOR3);
            break;
        case KEY_RIGHT:
            Serial.println(" phai");
            DcMotor2.run(95, MOTOR2);
            DcMotor3.run(-95, MOTOR3);
            break;
        default:
            Serial.println(" stop");
            DcMotor2.run(0, MOTOR2);
            DcMotor3.run(0, MOTOR3);
            break;
      }
      IR.resume();  
  }

}
