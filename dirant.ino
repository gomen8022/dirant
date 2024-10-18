#include <Servo.h>
#include <Timer.h>

#define SIGMOID             

#define SIGMOID_SLOPE       1
#define SIGMOID_OFFSET      4

#if !defined(EXPONENTIAL) ^ !defined(SIGMOID) ^ !defined(PROPORTIONAL) ^ !defined(RELATIVE)


#define LEFT_RSSI_PIN       0     // analog RSSI
#define RIGHT_RSSI_PIN      1
#define PAN_SERVO_PIN       4     // servo pin
#define RSSI_OFFSET_RIGHT   0
#define RSSI_OFFSET_LEFT    0


#define RSSI_MAX            400
#define RSSI_MIN            120


#define SERVO_MAX           180
#define SERVO_MIN           0


#define SERVO_MAX_STEP      5
#define SERVO_MIN_STEP      0.09    


#define DEADBAND            5


#define SERVO_DIRECTION     1

#define FIR_SIZE            10
#define LAST                FIR_SIZE - 1


uint16_t rssi_left_array[FIR_SIZE];
uint16_t rssi_right_array[FIR_SIZE];

float anglePan = 90;
boolean debug = false;

Timer timer;
Servo servoPan;


void setup() {
  servoPan.attach(PAN_SERVO_PIN);
  servoPan.write(90);

  for (int i = 0; i < FIR_SIZE; i++) {
    rssi_right_array[i] = 0;
    rssi_left_array[i] = 0;
  }

  Serial.begin(115200);
  while (!debug) {
    delay(3000);
    debug = true;
  }


  timer.every(50, mainLoop);
  timer.every(5, measureRSSI);
}


void loop() {

  timer.update();

}


void mainLoop() {

  uint16_t avgLeft = max(avg(rssi_left_array, FIR_SIZE) + RSSI_OFFSET_LEFT, RSSI_MIN);
  uint16_t avgRight = max(avg(rssi_right_array, FIR_SIZE) + RSSI_OFFSET_RIGHT, RSSI_MIN);

  uint8_t dynamicDeadband = (float(avgRight + avgLeft) / 2 - RSSI_MIN) / (RSSI_MAX - RSSI_MIN) * DEADBAND;

  if (abs(avgRight - avgLeft) < dynamicDeadband ) {
    return;
  }

  float ang = 0;

  // move towards stronger signal
  if (avgRight > avgLeft) {
  
  #if defined(EXPONENTIAL)
    float x = float(avgRight - avgLeft);
    x = x * x / 500;
    ang = x * SERVO_DIRECTION * -1;
  #endif

  #if defined(RELATIVE)
    ang = float(avgRight / avgLeft) * (SERVO_DIRECTION * -1);
  #endif

  #if defined(SIGMOID)
    float x = float(avgRight - avgLeft) / 10;
    x = SERVO_MAX_STEP / (1+ exp(-SIGMOID_SLOPE * x + SIGMOID_OFFSET));
    ang = x * SERVO_DIRECTION * -1;
  #endif
    
  #if defined(PROPORTIONAL)
    float x = float(avgRight - avgLeft) / 10;
    ang = x * SERVO_DIRECTION * -1;  
  #endif
  }
  else {

  #if defined(EXPONENTIAL)
    float x = float(avgLeft - avgRight);
    x = x * x / 500;
    ang = x * SERVO_DIRECTION;
  #endif

  #if defined(RELATIVE)
    ang = float(avgLeft / avgRight) * SERVO_DIRECTION;
  #endif

  #if defined(SIGMOID)
    float x = float(avgLeft - avgRight) / 10;
    x = SERVO_MAX_STEP / (1+ exp(-SIGMOID_SLOPE * x + SIGMOID_OFFSET));
    ang = x * SERVO_DIRECTION;
  #endif
    
  #if defined(PROPORTIONAL)
    float x = float(avgLeft - avgRight) / 10;
    ang = x * SERVO_DIRECTION;  
  #endif
  }

  // верхня і нижня межа кроку кута
  ang = (abs(ang) > SERVO_MAX_STEP ? SERVO_MAX_STEP * ang/abs(ang) : ang);
  ang = (abs(ang) < SERVO_MIN_STEP ? 0 : ang);

  // перемістити серво на n градусів
  movePanBy(ang);


  if (debug) {
   Serial.print("RSSI%: ");
   Serial.print(map(avgLeft, RSSI_MIN, RSSI_MAX, 0, 100));
   Serial.print(", ");
   Serial.print(map(avgRight, RSSI_MIN, RSSI_MAX, 0, 100));

    Serial.print("Calibration - left: ");
    Serial.print(avgLeft);
    Serial.print(" right: ");
    Serial.print(avgRight);

    Serial.print(" servo-angle: ");
    Serial.println(anglePan);
  }
}

void movePanBy(float angle) {

  anglePan += angle;
  anglePan = limit(SERVO_MIN, SERVO_MAX, anglePan);
  servoPan.write(anglePan);
}

void measureRSSI() {

  advanceArray(rssi_left_array, FIR_SIZE);
  advanceArray(rssi_right_array, FIR_SIZE);

  rssi_left_array[LAST] = analogRead(LEFT_RSSI_PIN);
  rssi_right_array[LAST] = analogRead(RIGHT_RSSI_PIN);
}

uint16_t avg(uint16_t samples[], uint8_t n) {

  uint32_t summ = 0;
  for (uint8_t i = 0; i < n; i++) {
    summ += samples[i];
  }

  return uint16_t(summ / n);
}

void advanceArray(uint16_t *samples, uint8_t n) {

  for (uint8_t i = 0; i < n - 1; i++) {
    samples[i] = samples[i + 1];
  }
}


float limit(float lowerLimit, float upperLimit, float var) {

  return min(max(var, lowerLimit), upperLimit);
}
