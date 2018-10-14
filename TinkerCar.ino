#include "Servo.h"
#include "FlySkyIBus.h" // https://github.com/aanon4/FlySkyIBus

#define MOTOR_MID               89
#define SERVO_MID               90
#define SERVO_F_MAX_ANGLE       160 // 90 - 180
#define SERVO_F_MIN_ANGLE       20  // 0 - 90
#define SERVO_B_MAX_ANGLE       110 // 90 - 180
#define SERVO_B_MIN_ANGLE       70  // 0 - 90
#define MOTOR_MAX_FORWARD       130 // 90 - 180
#define MOTOR_MAX_BACKWARD      80  // 0 - 90
//#define SUPER_RATE            1.8
#define THROTTLE_LOWPASS        63  // 2 ** n -1

Servo motorR;
Servo motorL;
Servo servoFront;
Servo servoBack;

enum {
  DISARMED,
  ARMED,
  FAILSAFE
};

int state;
int prev_throttle;
//long min, max;

void setup() 
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  
  Serial.begin(115200);
  IBus.begin(Serial);

  motorR.attach(9, 1000, 2000);
  motorL.attach(10, 1000, 2000);
  servoFront.attach(11, 1000, 2000);
  servoBack.attach(3, 1000, 2000);

  motorR.write(MOTOR_MID);
  motorL.write(MOTOR_MID);
  servoFront.write(SERVO_MID);
  servoBack.write(SERVO_MID);

  state = DISARMED;
  prev_throttle = 1000;

  //max = pow(500, SUPER_RATE);
  //min = -max;
}

void loop() 
{
  IBus.loop();
  int steering = IBus.readChannel(0);
  int throttle = IBus.readChannel(2);
  int diff = IBus.readChannel(3);
  int dir = IBus.readChannel(4);
  int arm = IBus.readChannel(5);
  int knob = IBus.readChannel(6);
  int swC = IBus.readChannel(7);

  if(arm < 1000) {
    state = FAILSAFE;
  }
  else if(arm < 1300) {
    state = DISARMED;
  }
  else if(arm >= 1300) {
    state = ARMED;
  }

  if(state == ARMED) {   
    int strength = map(knob, 1000, 2000, 64, 2);
    int modifier = (1500 - steering) / strength;

    if(swC > 1500) {
      // apply lowpass filter on throttle value
      // simple, fast, integer filter: http://www.microchip.com/forums/m108853.aspx
      throttle = ((long)THROTTLE_LOWPASS * prev_throttle + throttle) / (THROTTLE_LOWPASS + 1);
      prev_throttle = throttle;
    }

    int throttleL = throttle - modifier;
    int throttleR = throttle + modifier;

    // forward
    if(dir > 1700) {
      motorL.write(map(throttleL, 1000, 2000, MOTOR_MID, MOTOR_MAX_FORWARD));
      motorR.write(map(throttleR, 1000, 2000, MOTOR_MID, MOTOR_MAX_FORWARD));
    }
    // reverse
    else if(dir < 1300) {
      motorL.write(map(throttleL, 1000, 2000, MOTOR_MID, MOTOR_MAX_BACKWARD));
      motorR.write(map(throttleR, 1000, 2000, MOTOR_MID, MOTOR_MAX_BACKWARD));
    }
    // neutral
    else {
      motorL.write(MOTOR_MID);
      motorR.write(MOTOR_MID);
    }

    // makes steering non-linear, less sensitive close to mid-stick position
    //double expoFront = steering - 1500;
    //double expoBack = (steering - 1500) / 2;
    
    //expoFront = pow(abs(expoFront), SUPER_RATE);
    //expoBack = pow(abs(expoBack), SUPER_RATE);
    
    //if(steering < 1500) {
    //  expoFront = -expoFront;
    //  expoBack = -expoBack;
    //}

    servoFront.write(map(steering, 1000, 2000, SERVO_F_MIN_ANGLE, SERVO_F_MAX_ANGLE));
    servoBack.write(map(steering, 1000, 2000, SERVO_B_MIN_ANGLE, SERVO_B_MAX_ANGLE));
  }
  else {
    servoFront.write(SERVO_MID);
    servoBack.write(SERVO_MID);
    motorL.write(MOTOR_MID);
    motorR.write(MOTOR_MID);
  }

}

