#include "Servo.h"
#include "FlySkyIBus.h" // https://github.com/aanon4/FlySkyIBus

#define MOTOR_MID         88
#define SERVO_MID         90
#define SUPER_RATE        1.8
#define THROTTLE_LOWPASS  63  // 2 ** n -1

Servo motorR;
Servo motorL;
Servo servo;

enum {
  DISARMED,
  ARMED,
  FAILSAFE
};

int state;
int prev_throttle;
long min, max;

void setup() 
{
  Serial.begin(115200);
  IBus.begin(Serial);

  motorR.attach(9, 1000, 2000);
  motorL.attach(10, 1000, 2000);
  servo.attach(11, 1000, 2000);

  motorR.write(MOTOR_MID);
  motorL.write(MOTOR_MID);
  servo.write(SERVO_MID);

  state = DISARMED;
  prev_throttle = 0;

  max = pow(500, SUPER_RATE);
  min = -max;
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
      motorL.write(map(throttleL, 1000, 2000, MOTOR_MID, 180));
      motorR.write(map(throttleR, 1000, 2000, MOTOR_MID, 180));
    }
    // reverse
    else if(dir < 1300) {
      motorL.write(map(throttleL, 1000, 2000, MOTOR_MID, 0));
      motorR.write(map(throttleR, 1000, 2000, MOTOR_MID, 0));
    }
    // neutral
    else {
      motorL.write(MOTOR_MID);
      motorR.write(MOTOR_MID);
    }

    // makes steering non-linear, less sensitive close to mid-stick position
    double expo = steering - 1500;
    expo = pow(abs(expo), SUPER_RATE);
    if(steering < 1500) expo = -expo;
    
    servo.write(map(expo, min, max, 10, 170));
  }
  else {
    servo.write(SERVO_MID);
    motorL.write(MOTOR_MID);
    motorR.write(MOTOR_MID);
  }

}

