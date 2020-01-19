#define TRIG_PIN 16
#define ECHO_PIN 17
#define TIME_OUT 5000
void setup() {
  // put your setup code here, to run once:
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.begin(115200);
    //Serial2.begin(115200, SERIAL_8N1, 16, 17);
    Serial.println("setup finish");
}

float get_distance()
{
  long duration;
  float distance;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, LOW, TIME_OUT);
  distance = (float)duration / 36.9 / 2;
  return distance;
  
}

int loop_num = 0;
void loop() {
  float distance, sum_dist = 0;
  
  for (loop_num = 0; loop_num < 20; loop_num++) {
    sum_dist += get_distance();
    delay(6);
  }
  distance = sum_dist / 20;
  distance = (distance - 5)/0.7+2.5;
  sum_dist = 0;

  //Serial.println(distance);
  if (distance <= 0) {
    Serial.println("Echo time out !!"); // nếu thời gian phản hồi vượt quá Time_out của hàm pulseIn
  }
  else {
    // Hiển thị khoảng cách đo được lên Serial Monitor   
    //Serial.print("Distance to nearest obstacle (cm): ");
    Serial.println(distance);
  }
  // Chờ 1s và lặp lại cu kỳ trên
  delay(10);

}
