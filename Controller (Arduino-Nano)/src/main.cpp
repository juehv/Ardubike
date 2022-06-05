#include <Arduino.h>
#define DEBUG_OUTPUT

#define PIN_SENS_BREAK 3
#define PIN_SENS_PEDAL 2 // Interrupt Pins 2, 3
#define PIN_SENS_SPEED 5 // Interrupt Pins 2, 3
#define PIN_SENS_BAT A7

#define PIN_PWM_VESC 9 // PWM Pins 3, 5, 6, 9, 10, 11

#define PIN_LOW_BAT LED_BUILTIN // internal LED

#define PEDAL_COUNTER_BUFFER_SIZE 12 // size of buffer for calculating pedalling speed

#define PID_FREQUENCE_HZ 10
#define PID_SETPOINT_PEDAL_SPEED 120 //in turns per second (115 ~~ 25 km/h)
#define PID_FREQUENCE_TIME (1000/PID_FREQUENCE_HZ) //every 100 ms = 10 Hz

#include <FastPID.h> //https://github.com/mike-matera/FastPID
#include <Servo.h> //https://www.arduino.cc/reference/en/libraries/servo/

uint32_t prevMills = 0;

uint8_t pedalStopCounter = 0; // 0 if pedaling; counting how much cycles in a row not pedaling
volatile uint16_t pedalCount = 0; // counts pedal pulses in ISR


uint16_t pedalSpeed = 0; // roghly turns per second (U/s) with two digit fixed point
volatile uint8_t pedalCounterBufferPos = 0;
volatile uint16_t* pedalCounterBuffer = new uint16_t[PEDAL_COUNTER_BUFFER_SIZE];
volatile uint32_t pedalCounterPreviousValue = 0;

volatile uint8_t breaking = 0;

int16_t motorSpeedValue = 0;
int16_t motorSpeedValueTarget = 0;

uint16_t pidEvalTime = 0; // time since last pid evaluation
//FastPID(float kp, float ki, float kd, float hz, int bits=16, bool sign=false)
//https://github.com/mike-matera/FastPID
FastPID motorSpeedPid(0.1f, 0.5f, 0.0f, PID_FREQUENCE_HZ, 8, false);

Servo myservo; 

void isrBreak() {
  breaking = 1;
  // ensure fast break reaction
  myservo.write(0);
}

void isrSpeedCounter() {
  // TODO implement identical to pedal counter
}

void isrPedalCounter() {
  uint32_t tmpMillis = millis();

  // clear buffer from old values
  // 20 seconds stop time allowed
  if (pedalCounterPreviousValue == 0 ||
      (tmpMillis - pedalCounterPreviousValue) > 20000) {
    for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++) {
      pedalCounterBuffer[i] = 0;
    }
    pedalCounterBufferPos = 0;
    pedalCounterPreviousValue = tmpMillis;
    return;
  }

  // calc timestamp diff
  uint16_t tmpDiff = abs(tmpMillis - pedalCounterPreviousValue);

  // filter bouncing and exit
  if (tmpDiff < 30) { // 28ms = 3 turns per second for 12 pulses per revolution
    return;
  } else {
    pedalCounterBuffer[pedalCounterBufferPos] = tmpDiff;
  }

  // update position pointer
  pedalCounterBufferPos++;
  if (pedalCounterBufferPos >= PEDAL_COUNTER_BUFFER_SIZE) {
    pedalCounterBufferPos = 0;
  }
  pedalCounterPreviousValue = tmpMillis;
}

inline void caclucatePedalSpeed() {
  // caluclate pedalling speed (contiuus version)
  // buffer holds timestamps with measured sensor pulses
  // 12 pulses per revolution
  // algorithm: calc avg time between pulses - use value for turns per second calculation - use simple low pass filter on output variable

  // if values are old skip calculation
  // TODO don't know what the right threshold is for registering stop paddelling
  if (pedalCounterPreviousValue == 0 ||
      (millis() - pedalCounterPreviousValue) > 500) {
    pedalSpeed = 0;
    return;
  }

  // calculate AVG time between pulses
  uint8_t noOfUsedSlots = 0;
  uint16_t tmpAVG = 0;
  noInterrupts();
  for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++) {
    if (pedalCounterBuffer[i] > 0) {
      noOfUsedSlots++;
      tmpAVG += pedalCounterBuffer[i];
    } else {
      // as buffer is build up from 0 to sizeOf, we can stop when we find empty slots
      break;
    }
  }
  interrupts();

  // calculate turns per second
  if (noOfUsedSlots == 0) {
    // not pedalling
    pedalSpeed = 0;
  } else if (noOfUsedSlots > 2) {
    tmpAVG /= noOfUsedSlots;
    if (pedalSpeed > 1) {
      // low pass filter
      pedalSpeed += (100000 / (tmpAVG * 12));
      pedalSpeed /= 2;
    } else {
      // no filtering
      pedalSpeed = (100000 / (tmpAVG * 12));
    }
  } else {
    // just started pedaling --> start support
    pedalSpeed = 1;
  }
}

inline void calculateMotorSpeedTarget(uint8_t setpointPedalSpeed) {
  // calculate motor speed value (--> PID contoller)
  if (pedalSpeed == 0) {
    // no pedalling -> no motor
    motorSpeedValueTarget = 0;
    motorSpeedPid.clear();
  } else if (pedalSpeed >= 170) { // 130??
    // pedalling too fast -> exceeding 25 km/h
    //motorSpeedValueTarget = 0;
    //motorSpeedPid.clear();
  } else if (pedalSpeed < setpointPedalSpeed * 0.7) {
    // far under setpoint, just turn on motor
    motorSpeedValueTarget = 255;
    motorSpeedPid.clear();
    //  } else if (pedalSpeed > setpointPedalSpeed * 1.5) {
    //    // far over setpoint, just turn off motor
    //    motorSpeedValueTarget = 0;
    //    motorSpeedPid.clear();
  } else {
    // optimize with pid
    motorSpeedValueTarget = motorSpeedPid.step(setpointPedalSpeed , pedalSpeed); // support for 1 turn per second
  }


  // TODO to be removed later (just for testing max power)
//  if (motorSpeedValueTarget > 0)
//    motorSpeedValueTarget = 255;

}

inline void calculateMotorSpeed() {
  motorSpeedValue = map(motorSpeedValueTarget, 0, 255, 0, 180);
  // // calculate motor speed value from target

  // // case for breaking
  // if (motorSpeedValueTarget == 0 || breaking == 0) {
  //   motorSpeedValue = 0;
  //   return;
  // }

  // // case for close to target
  // if (motorSpeedValue > motorSpeedValueTarget - 20 ||
  //     motorSpeedValue < motorSpeedValueTarget + 20) {
  //   motorSpeedValue = motorSpeedValueTarget;
  //   return;
  // }

  // // case for ramping
  // if (motorSpeedValue < motorSpeedValueTarget) {
  //   motorSpeedValue += 20;
  // } else if (motorSpeedValue > motorSpeedValueTarget) {
  //   motorSpeedValue -= 20;
  // }

  // if (motorSpeedValue > 220) {
  //   motorSpeedValue = 255;
  // } else if (motorSpeedValue < 50) {
  //   motorSpeedValue = 50;
  // }
}

void setup() {
  Serial.begin(115200);

  myservo.attach(PIN_PWM_VESC);
  myservo.write(0);

  pinMode(PIN_SENS_BREAK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SENS_BREAK), isrBreak, FALLING);

  pinMode(PIN_SENS_PEDAL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SENS_PEDAL), isrPedalCounter, FALLING);

  pinMode(PIN_SENS_SPEED, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(PIN_SENS_SPEED), isrSpeedCounter, FALLING);


  pinMode(PIN_LOW_BAT, OUTPUT);
  digitalWrite(PIN_LOW_BAT, LOW); // LED OFF

  if (motorSpeedPid.err()) {
    Serial.println("PID faild to inizialize (config error)!");
    for (;;) {}
  }

  Serial.println("Setup done.");
}

void resetEverything() {
  // Stop Engine
  myservo.write(0);

  // reset PID
  motorSpeedPid.clear();

  // reset motor variables
  motorSpeedValue = 0;
  motorSpeedValueTarget = 0;

  // pedal speed buffer
  for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++) {
    pedalCounterBuffer[i] = 0;
  }
  pedalCounterBufferPos = 0;
}


void loop() {
  // schedule time tracking
  uint16_t loopRunTime = millis() - prevMills;
  pidEvalTime += loopRunTime;
  prevMills = millis();

  // fast breaking reaction (and skip the rest)
  // breaking = digitalRead(PIN_SENS_BREAK);
  // if (breaking == 0) {
  //   digitalWrite(PIN_PWM_BREAK, HIGH);
  //   resetEverything();
  //   return;
  // } else {
  //   digitalWrite(PIN_PWM_BREAK, LOW);
  // }

  // calculate motor support and set motor
  if (pidEvalTime >= PID_FREQUENCE_TIME) {
    caclucatePedalSpeed();
    calculateMotorSpeedTarget(PID_SETPOINT_PEDAL_SPEED);
    calculateMotorSpeed();
  }

  if (pedalSpeed > 0) {
    myservo.write(motorSpeedValue);
  } else {
    myservo.write(0);
  }

  // reset eval time for 10 Hz loop
  if (pidEvalTime >= 100) {
    pidEvalTime = 0;
  }

  // check battery
  uint16_t batVoltage = analogRead(PIN_SENS_BAT);
  batVoltage *= 55; // calculate voltage (defined by voltage devider
  batVoltage /= 100; // make number smaller (XX.X Volt fixed point)
  if (batVoltage < 370) {
    // led acid 37V
    digitalWrite(PIN_LOW_BAT, HIGH);
  }

  //TODO voltage korrektur
  // voltage / target voltage => multiplyer for motor target

#ifdef DEBUG_OUTPUT
  // debug output
  //delay(50);
  Serial.print("speed:");
  Serial.print(motorSpeedValue);
  Serial.print(", speedTarget:");
  Serial.print(motorSpeedValueTarget);
  //  Serial.print(", PedalCounter:");
  //  Serial.print(pedalCount);
  Serial.print(", pedal:");
  Serial.print(pedalSpeed);
  // Serial.print(", Break:");
  // Serial.print(breaking);
  Serial.print(", Batt:");
  Serial.print(batVoltage);
  //Serial.print(", LoopTime:");
  //Serial.print(loopRunTime);
  //  Serial.print(", PIDTime:");
  //  Serial.print(pidEvalTime);
  //  Serial.print(", PedalBufferPosition:");
  //  Serial.print(pedalCounterBufferPos);
  //  Serial.print(", PedalBuffer:[");
  //  for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++) {
  //    Serial.print(pedalCounterBuffer[i]);
  //    Serial.print(", ");
  //  }
  //  Serial.print("]");
  Serial.println("");
#endif
}