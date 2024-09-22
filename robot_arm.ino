// C++ code
#include <Arduino_LED_Matrix.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define minMap 1
#define maxMap 8

// 230 to 950 measured
#define potMin 300
#define potMax 850

#define SERVO_MIN 60  
#define SERVO_MAX 580

#define FREQUENCY 60


//uint8_t servonum = 0;
int servoCount = 4;

int potValues[4] = {0, 0, 0, 0};

int pot1 = 0;
int pot2 = 0;
int pot3 = 0;
int pot4 = 0;

ArduinoLEDMatrix matrix;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);


int readPot(int input){
  int value = analogRead(input);
  if(value < potMin) value = potMin;
  if(value > potMax) value = potMax;
  return value;
}

void moveServo(int control, int motor){
  int prev_Value = potValues[motor];

  if((abs(control - prev_Value) > 5)){
    pwm.setPWM(motor, 0, control);
    potValues[motor] = control;
  }
}

void moveServos(int servoCount){
  for(int i = 0; i < servoCount; i ++){
    int pin = A0 + i;
    moveServo(map(readPot(pin), potMin, potMax, SERVO_MAX, SERVO_MIN), i);
    //moveServo(map(analogRead(pin), 0, 1023, SERVO_MIN, SERVO_MAX), i);
  }
}

void moveMiddle(int servoCount){
  for(int i = 0; i < servoCount; i ++){  
    pwm.setPWM(i, 0, ((SERVO_MAX + SERVO_MIN) / 2));
  }
}

// the middle value is 511
void calibratePots(){
  Serial.print("A0: ");
  Serial.println(analogRead(A0));
  Serial.print("A1: ");
  Serial.println(analogRead(A1));
  Serial.print("A2: ");
  Serial.println(analogRead(A2));  
  Serial.print("A3: ");
  Serial.println(analogRead(A3));

  delay(50);
}

void copyMove(){
  moveServos(servoCount);
  delay(30);

  byte frame[8][12] = {
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
  };

  pot1 = map(potValues[0], SERVO_MIN, SERVO_MAX, minMap, maxMap);
  pot2 = map(potValues[1], SERVO_MIN, SERVO_MAX, minMap, maxMap);
  pot3 = map(potValues[2], SERVO_MIN, SERVO_MAX, minMap, maxMap);
  pot4 = map(potValues[3], SERVO_MIN, SERVO_MAX, minMap, maxMap);

  // Display the values on the LED matrix
  for (int i = 0; i <= pot1; i++) {
    frame[8-i][0] = 1;
  }
  for (int i = 0; i <= pot2; i++) {
    frame[8-i][1] = 1;
  }
  for (int i = 0; i <= pot3; i++) {
    frame[8-i][2] = 1;
  }
  for (int i = 0; i <= pot4; i++) {
    frame[8-i][3] = 1;
  }

  matrix.renderBitmap(frame, 8, 12);

  //delay(30);
}

void setup(){
  matrix.begin();

  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  Serial.begin(9600);
}

void loop(){
  //moveMiddle(servoCount);
  //calibratePots();
  copyMove();

}
