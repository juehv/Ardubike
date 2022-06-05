#include <Arduino.h>
#define DEBUG_OUTPUT

#define PIN_SENS_BREAK 2 // motor is not able to break .. will not be used
#define PIN_SENS_PEDAL 3 // Interrupt Pins 2, 3
#define PIN_SENS_SPEED 5 // Interrupt Pins 2, 3
#define PIN_SENS_BAT A7

#define PIN_PWM_VESC 9 // PWM Pins 3, 5, 6, 9, 10, 11

#define PIN_LOW_BAT LED_BUILTIN // internal LED

#define PEDAL_COUNTER_PULSES_PER_TURN 12
#define PEDAL_COUNTER_BUFFER_SIZE 12 // size of buffer for calculating pedalling speed

#define PID_FREQUENCE_HZ 10
#define PID_SETPOINT_PEDAL_SPEED 100 //in turns per second + two digit fixed point (115 ~~ 25 km/h for the testbike)
#define PID_FREQUENCE_TIME (1000/PID_FREQUENCE_HZ) //every 100 ms = 10 Hz

#include <FastPID.h> //https://github.com/mike-matera/FastPID
#include <Servo.h> //https://www.arduino.cc/reference/en/libraries/servo/

uint32_t prevMills = 0;

uint8_t pedalStopCounter = 0; // 0 if pedaling; counting how much cycles in a row not pedaling
volatile uint16_t pedalCount = 0; // counts pedal pulses in ISR

uint16_t pedalSpeed = 0; // roghly turns per second (U/s) with two digit fixed point (*100)
volatile uint8_t pedalCounterBufferPos = 0;
volatile uint16_t* pedalCounterBuffer = new uint16_t[PEDAL_COUNTER_BUFFER_SIZE];
volatile uint32_t pedalCounterPreviousValue = 0;

int16_t motorSpeedValue = 0;
int16_t motorSpeedValueTarget = 0;

//FastPID(float kp, float ki, float kd, float hz, int bits=16, bool sign=false)
//https://github.com/mike-matera/FastPID
FastPID motorSpeedPid(0.1f, 0.5f, 0.0f, PID_FREQUENCE_HZ, 8, false);

Servo myservo; 

void resetEverything();

void isrPedalCounter() {
  uint32_t tmpMillis = millis();

  // clear buffer from old values
  // 20 seconds stop time allowed without clearing
  if (pedalCounterPreviousValue == 0 ||
      (tmpMillis - pedalCounterPreviousValue) > 3000) {
    for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++) {
      pedalCounterBuffer[i] = 0;
    }
    pedalCounterBufferPos = 0;
    pedalCounterPreviousValue = tmpMillis;
    resetEverything();
    return;
  }

  // calc timestamp diff
  uint16_t tmpDiff = abs(tmpMillis - pedalCounterPreviousValue);
Serial.println(tmpDiff);
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
      pedalSpeed += (100000 / (tmpAVG * PEDAL_COUNTER_PULSES_PER_TURN));
      pedalSpeed /= 2;
    } else {
      // no filtering
      pedalSpeed = (100000 / (tmpAVG * PEDAL_COUNTER_PULSES_PER_TURN));
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
  } else if (pedalSpeed < setpointPedalSpeed * 0.5) {
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
  
  motorSpeedValue = map(motorSpeedValueTarget, 0, 255, 0, 180);
}

void setup() {
  Serial.begin(115200);

  myservo.attach(PIN_PWM_VESC);
  myservo.write(0);

  // pinMode(PIN_SENS_BREAK, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_SENS_BREAK), isrBreak, FALLING);

  pinMode(PIN_SENS_PEDAL, INPUT_PULLUP);
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
  // wait for next round
  uint16_t loopRunTime;
  uint16_t loopUtilization = (millis() - prevMills) * 100 / PID_FREQUENCE_TIME;
  while ((loopRunTime = millis() - prevMills) < PID_FREQUENCE_TIME){
    _delay_ms(5);
  }
  prevMills = millis();

  // calculate motor support and set motor
  caclucatePedalSpeed();
  calculateMotorSpeedTarget(PID_SETPOINT_PEDAL_SPEED);
  
  if (pedalSpeed > 0) {
    myservo.write(motorSpeedValue);
  } else {
    myservo.write(0);
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
  Serial.print(", pedalSpeed:");
  Serial.print(pedalSpeed);
  // Serial.print(", Break:");
  // Serial.print(breaking);
  Serial.print(", Batt:");
  Serial.print(batVoltage);
  //Serial.print(", LoopTime:");
  //Serial.print(loopRunTime);
  Serial.print(", LoopUtilization:");
  Serial.print(loopUtilization);
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

//TODO transmit information to esp32, read input from esp32
}