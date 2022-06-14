/*
MIT License

Copyright (c) 2021 Jens Heuschkel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include <Arduino.h>
//#define DEBUG_OUTPUT

#define PIN_SENS_PEDAL 3 // Interrupt Pins 2, 3
#define PIN_SENS_SPEED 5 // Interrupt Pins 2, 3
#define PIN_SENS_BAT A7

#define PIN_PWM_VESC 9 // PWM Pins 3, 5, 6, 9, 10, 11

#define PIN_LOW_BAT LED_BUILTIN // internal LED

#define PEDAL_COUNTER_PULSES_PER_TURN 12
#define PEDAL_COUNTER_BUFFER_SIZE 12 // size of buffer for calculating pedalling speed

#define PID_FREQUENCE_HZ 10
#define PID_SETPOINT_PEDAL_SPEED 200                 // in turns per second + two digit fixed point (200 = 110 turns per minute)
#define PID_FREQUENCE_TIME (1000 / PID_FREQUENCE_HZ) // every 100 ms = 10 Hz

#define SOFTSERIAL_RX_PIN 2
#define SOFTSERIAL_TX_PIN A0

#include <FastPID.h>        //https://github.com/mike-matera/FastPID
#include <Servo.h>          //https://www.arduino.cc/reference/en/libraries/servo/
#include <SoftwareSerial.h> //https://docs.arduino.cc/learn/built-in-libraries/software-serial
#include <TinyGPS++.h>      //https://github.com/mikalhart/TinyGPSPlus

uint32_t prevMills = 0;

uint8_t pedalStopCounter = 0;     // 0 if pedaling; counting how much cycles in a row not pedaling
volatile uint16_t pedalCount = 0; // counts pedal pulses in ISR

uint16_t pedalSpeed = 0; // roghly turns per second (U/s) with two digit fixed point (*100)
volatile uint8_t pedalCounterBufferPos = 0;
volatile uint16_t *pedalCounterBuffer = new uint16_t[PEDAL_COUNTER_BUFFER_SIZE];
volatile uint32_t pedalCounterPreviousValue = 0;

int16_t motorSpeedValue = 0;
int16_t motorSpeedValueTarget = 0;

uint16_t batVoltage;

// FastPID(float kp, float ki, float kd, float hz, int bits=16, bool sign=false)
// https://github.com/mike-matera/FastPID
FastPID motorSpeedPid(0.1f, 0.5f, 0.0f, PID_FREQUENCE_HZ, 8, false);

Servo motorSpeedOutputPWM;

// Set up a new SoftwareSerial object
SoftwareSerial sSerial(SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN);
TinyGPSPlus gps;
bool gpsLocked = false;
double bikeSpeedMPS = 0;

void resetEverything();

void isrPedalCounter()
{
  uint32_t tmpMillis = millis();

  // clear buffer from old values
  // 20 seconds stop time allowed without clearing
  if (pedalCounterPreviousValue == 0 ||
      (tmpMillis - pedalCounterPreviousValue) > 3000)
  {
    for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++)
    {
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
  if (tmpDiff < 30)
  { // 28ms = 3 turns per second for 12 pulses per revolution
    return;
  }
  else
  {
    pedalCounterBuffer[pedalCounterBufferPos] = tmpDiff;
  }

  // update position pointer
  pedalCounterBufferPos++;
  if (pedalCounterBufferPos >= PEDAL_COUNTER_BUFFER_SIZE)
  {
    pedalCounterBufferPos = 0;
  }
  pedalCounterPreviousValue = tmpMillis;
}

inline void caclucatePedalSpeed()
{
  // caluclate pedalling speed (contiuus version)
  // buffer holds timestamps with measured sensor pulses
  // 12 pulses per revolution
  // algorithm: calc avg time between pulses - use value for turns per second calculation - use simple low pass filter on output variable

  // if values are old skip calculation
  // TODO don't know what the right threshold is for registering stop paddelling
  if (pedalCounterPreviousValue == 0 ||
      (millis() - pedalCounterPreviousValue) > 250)
  {
    pedalSpeed = 0;
    return;
  }

  // calculate AVG time between pulses
  uint8_t noOfUsedSlots = 0;
  uint16_t tmpAVG = 0;
  // noInterrupts();
  for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++)
  {
    if (pedalCounterBuffer[i] > 0)
    {
      noOfUsedSlots++;
      tmpAVG += pedalCounterBuffer[i];
    }
    else
    {
      // as buffer is build up from 0 to sizeOf, we can stop when we find empty slots
      break;
    }
  }
  // interrupts();

  // calculate turns per second
  if (noOfUsedSlots == 0)
  {
    // not pedalling
    pedalSpeed = 0;
  }
  else if (noOfUsedSlots > 2)
  {
    tmpAVG /= noOfUsedSlots;
    if (pedalSpeed > 1)
    {
      // low pass filter
      pedalSpeed += (100000 / (tmpAVG * PEDAL_COUNTER_PULSES_PER_TURN));
      pedalSpeed /= 2;
    }
    else
    {
      // no filtering
      pedalSpeed = (100000 / (tmpAVG * PEDAL_COUNTER_PULSES_PER_TURN));
    }
  }
  else
  {
    // just started pedaling --> start support
    pedalSpeed = 1;
  }
}

inline void calculateMotorSpeedTarget(uint8_t setpointPedalSpeed)
{
  // calculate motor speed value (--> PID contoller)
  if (pedalSpeed == 0)
  {
    // no pedalling -> no motor
    motorSpeedValueTarget = 0;
    motorSpeedPid.clear();
  }
  else if (pedalSpeed < setpointPedalSpeed * 0.5)
  {
    // far under setpoint, just turn on motor
    motorSpeedValueTarget = 255;
    motorSpeedPid.clear();
    //  } else if (pedalSpeed > setpointPedalSpeed * 1.5) {
    //    // far over setpoint, just turn off motor
    //    motorSpeedValueTarget = 0;
    //    motorSpeedPid.clear();
  }
  else
  {
    // optimize with pid
    //motorSpeedValueTarget = motorSpeedPid.step(setpointPedalSpeed, pedalSpeed); // support for 1 turn per second
  
    // try a linear support after half cadence is reached
    motorSpeedValueTarget = 255 - map(pedalSpeed, setpointPedalSpeed * 0.5, setpointPedalSpeed, 0, 127);
  }

  // map motor speed to servo output, but only when gps is there and reduce performance to 1/2 if not
  motorSpeedValue = gpsLocked ? map(motorSpeedValueTarget, 0, 255, 0, 180) : map(motorSpeedValueTarget, 0, 255, 0, 90);
  
  // linear fade out when too fast (>25 km/h)
  if (bikeSpeedMPS > 6.5){
    long bikeSpeedAsLong = (long) bikeSpeedMPS * 100;
    motorSpeedValue *= map(bikeSpeedAsLong, 650, 750, 10, 00) * 0.1;
    motorSpeedValue = motorSpeedValue < 0 ? 0 : motorSpeedValue;
  }
}

void setup()
{
  Serial.begin(115200);

  // Define pin modes for soft serial TX and RX
  pinMode(SOFTSERIAL_RX_PIN, INPUT);
  pinMode(SOFTSERIAL_TX_PIN, OUTPUT);
  sSerial.begin(9600);

  motorSpeedOutputPWM.attach(PIN_PWM_VESC);
  motorSpeedOutputPWM.write(0);

  // pinMode(PIN_SENS_BREAK, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_SENS_BREAK), isrBreak, FALLING);

  pinMode(PIN_SENS_PEDAL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SENS_PEDAL), isrPedalCounter, FALLING);

  pinMode(PIN_SENS_SPEED, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_SENS_SPEED), isrSpeedCounter, FALLING);

  pinMode(PIN_LOW_BAT, OUTPUT);
  digitalWrite(PIN_LOW_BAT, LOW); // LED OFF

  if (motorSpeedPid.err())
  {
    Serial.println("PID faild to inizialize (config error)!");
    for (;;)
    {
    }
  }

  resetEverything();

  Serial.println("Setup done.");
}

void resetEverything()
{
  // Stop Engine
  motorSpeedOutputPWM.write(0);

  // reset PID
  motorSpeedPid.clear();

  // reset motor variables
  motorSpeedValue = 0;
  motorSpeedValueTarget = 0;

  // pedal speed buffer
  for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++)
  {
    pedalCounterBuffer[i] = 0;
  }
  pedalCounterBufferPos = 0;
}

void loop()
{
  // {
  //   // init pwm -> vesc (comment out for normal operation)
  //   motorSpeedOutputPWM.write(0);
  //   _delay_ms(1000);
  //   motorSpeedOutputPWM.write(180);
  //   _delay_ms(1000);
  //   return;
  // }

  // wait for next round
  uint16_t loopRunTime;
  uint16_t loopUtilization = (millis() - prevMills) * 100 / PID_FREQUENCE_TIME;
  while ((loopRunTime = millis() - prevMills) < PID_FREQUENCE_TIME)
  {
    //_delay_ms(5);
  }
  prevMills = millis();

  // check for gps data
  while (sSerial.available())
  {
    if (gps.encode(sSerial.read())) // encode gps data
    {
#ifdef DEBUG_OUTPUT
      Serial.print("SATS: ");
      Serial.println(gps.satellites.value());
      Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LONG: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("ALT: ");
      Serial.println(gps.altitude.meters());
      Serial.print("SPEED: ");
      Serial.println(gps.speed.mps());

      Serial.print("Date: ");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.println(gps.date.year());

      Serial.print("Hour: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
      Serial.println("---------------------------");
#endif

      // signal available?
      gpsLocked = gps.satellites.value() > 0;

      // push data to datalogger
      // sSerial.print("SAT");
      sSerial.print(gps.satellites.value());
      sSerial.print(",");
      sSerial.print(gps.date.day());
      sSerial.print(".");
      sSerial.print(gps.date.month());
      sSerial.print(".");
      sSerial.print(gps.date.year());
      sSerial.print("-");
      sSerial.print(gps.time.hour());
      sSerial.print(":");
      sSerial.print(gps.time.minute());
      sSerial.print(":");
      sSerial.print(gps.time.second());
      // sSerial.print(", LAT: ");
      // sSerial.print(gps.location.lat(), 6);
      // sSerial.print(", LONG: ");
      // sSerial.print(gps.location.lng(), 6);
      // sSerial.print(", ALT: ");
      // sSerial.print(gps.altitude.meters());
      sSerial.print(",");
      bikeSpeedMPS = gps.speed.mps();
      sSerial.print(gps.speed.mps());

      sSerial.print(",");
      sSerial.print(motorSpeedValue);
      sSerial.print(",");
      sSerial.print(motorSpeedValueTarget);
      // sSerial.print(",");
      // sSerial.print(pedalCount);
      sSerial.print(",");
      sSerial.print(pedalSpeed);
      // sSerial.print(",");
      // sSerial.print(batVoltage);
      // sSerial.print(", LoopUtilization:");
      // sSerial.print(loopUtilization);
      sSerial.println("");
    }
  }

  // calculate motor support and set motor
  caclucatePedalSpeed();
  calculateMotorSpeedTarget(PID_SETPOINT_PEDAL_SPEED);

  if (pedalSpeed > 0)
  {
    motorSpeedOutputPWM.write(motorSpeedValue);
  }
  else
  {
    motorSpeedOutputPWM.write(0);
  }

  // check battery
  batVoltage = analogRead(PIN_SENS_BAT);
  batVoltage *= 55;  // calculate voltage (defined by voltage devider
  batVoltage /= 100; // make number smaller (XX.X Volt fixed point)
  if (batVoltage < 370)
  {
    // led acid 37V
    digitalWrite(PIN_LOW_BAT, HIGH);
  }

  // TODO voltage korrektur
  //  voltage / target voltage => multiplyer for motor target

#ifdef DEBUG_OUTPUT
  // debug output
  // delay(50);
  Serial.print("speed:");
  Serial.print(motorSpeedValue);
  Serial.print(", speedTarget:");
  Serial.print(motorSpeedValueTarget);
  Serial.print(", PedalCounter:");
  Serial.print(pedalCount);
  Serial.print(", pedalSpeed:");
  Serial.print(pedalSpeed);
  Serial.print(", Batt:");
  Serial.print(batVoltage);
  // Serial.print(", LoopTime:");
  // Serial.print(loopRunTime);
  Serial.print(", LoopUtilization:");
  Serial.print(loopUtilization);
  Serial.print(", PedalBufferPosition:");
  Serial.print(pedalCounterBufferPos);
  Serial.print(", PedalBuffer:[");
  for (uint8_t i = 0; i < PEDAL_COUNTER_BUFFER_SIZE; i++)
  {
    Serial.print(pedalCounterBuffer[i]);
    Serial.print(", ");
  }
  Serial.print("]");
  Serial.println("");
#endif

  // TODO transmit information to esp32, read input from esp32
}