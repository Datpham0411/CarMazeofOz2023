#include <Arduino.h>
#include <avr/interrupt.h>
#include "CarMazeofOz2023.h"

#define SOUND_SPEED 34320
#define MAX_PWM 255
#define MIN_PWM 0
#define STEP_TIME_64 4e-6
#define TIMER1_STEP_CYCLE 65536

const byte IN1HEAD = 4;
const byte IN2HEAD = 5;
const byte IN3HEAD = 6;
const byte IN4HEAD = 7;
const byte IN1TAIL = 0;
const byte IN2TAIL = 1;
const byte IN3TAIL = 12;
const byte IN4TAIL = 13;

const byte encLeft = 2;
const byte encRight = 3;
const byte trig = 8;
const byte echoHead = 9;
const byte echoLeft = 10;
const byte echoRight = 11;

volatile float distanceHead = 0, distanceLeft = 0, distanceRight = 0;
volatile float distanceHead_SAMPLE[11], distanceLeft_SAMPLE[11], distanceRight_SAMPLE[11];
volatile unsigned long currentSensorHead = 0, currentSensorLeft = 0, currentSensorRight = 0;
volatile byte allSensor = 0;
volatile bool headHigh = 0, leftHigh = 0, rightHigh = 0;
volatile bool headIsRec = 0;

carMazeOfOz::carMazeOfOz() {
  ;
}

void carMazeOfOz::setPin() {
  pinMode(IN1HEAD, OUTPUT);
  pinMode(IN2HEAD, OUTPUT);
  pinMode(IN3HEAD, OUTPUT);
  pinMode(IN4HEAD, OUTPUT);
  pinMode(IN1TAIL, OUTPUT);
  pinMode(IN2TAIL, OUTPUT);
  pinMode(IN3TAIL, OUTPUT);
  pinMode(IN4TAIL, OUTPUT);

  pinMode(encLeft, INPUT);
  pinMode(encRight, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echoHead, INPUT);
  pinMode(echoLeft, INPUT);
  pinMode(echoRight, INPUT);
}

void carMazeOfOz::setInterrupt() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;

  TCCR1B |= (1 << CS11) | (1 << CS10);
  TCNT1 = 0;
  TIMSK1 = (1 << TOIE1);

  PCICR = 0;
  PCMSK0 = 0;

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3);
  sei();

  digitalWrite(trig, LOW);
}

ISR (TIMER1_OVF_vect) {
  TCNT1 = 0;
}

ISR (PCINT0_vect) {
  if (digitalRead(echoHead) != headHigh) {
    if (headHigh){
      static byte i = 0;
      distanceHead_SAMPLE[10] -= distanceHead_SAMPLE[i];
      distanceHead_SAMPLE[i] = (TCNT1 - currentSensorHead + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
      distanceHead_SAMPLE[10] += distanceHead_SAMPLE[i];
      distanceHead = ((SOUND_SPEED * STEP_TIME_64) * (distanceHead_SAMPLE[10] / 10)) / 2;
      i = (i + 1) % 10;
      ++allSensor;
    } else {
      currentSensorHead = TCNT1;
    }
    headHigh = !headHigh;
  }
  if (digitalRead(echoLeft) != leftHigh) {
    if (leftHigh) {
      static byte i = 0;
      distanceLeft_SAMPLE[10] -= distanceLeft_SAMPLE[i];
      distanceLeft_SAMPLE[i] = (TCNT1 - currentSensorLeft + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
      distanceLeft_SAMPLE[10] += distanceLeft_SAMPLE[i];
      distanceLeft = ((SOUND_SPEED * STEP_TIME_64) * (distanceLeft_SAMPLE[10] / 10)) / 2;
      i = (i + 1) % 10;
      ++allSensor;
    } else {
      currentSensorLeft = TCNT1; 
    }
    leftHigh = !leftHigh;
  }
  if (digitalRead(echoRight) != rightHigh) {
    if (rightHigh) {
      static byte i = 0;
      distanceRight_SAMPLE[10] -= distanceRight_SAMPLE[i];
      distanceRight_SAMPLE[i] = (TCNT1 - currentSensorRight + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
      distanceRight_SAMPLE[10] += distanceRight_SAMPLE[i];
      distanceRight = ((SOUND_SPEED * STEP_TIME_64) * (distanceRight_SAMPLE[10] / 10)) / 2;
      i = (i + 1) % 10;
      ++allSensor;
    } else {
      currentSensorRight = TCNT1; 
    }
    rightHigh = !rightHigh;
  }
  if (allSensor == 3) {
    allSensor = 0;
    digitalWrite(trig, HIGH);
  }
}

void carMazeOfOz::setMotorLeftHEAD(byte speed, bool direction) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  digitalWrite(IN4HEAD, direction);
  analogWrite(IN3HEAD, abs(255*direction - speed));
}

void carMazeOfOz::setMotorLeftTAIL(byte speed, bool direction) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  digitalWrite(IN4TAIL, direction);
  analogWrite(IN3TAIL, abs(255*direction - speed));
}

void carMazeOfOz::setMotorRightHEAD(byte speed, bool direction) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  digitalWrite(IN1HEAD, direction);
  analogWrite(IN2HEAD, abs(255*direction - speed));
}

void carMazeOfOz::setMotorRightTAIL(byte speed, bool direction) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  digitalWrite(IN1TAIL, direction);
  analogWrite(IN2TAIL, abs(255*direction - speed));
}

void carMazeOfOz::setSpeedLeft(volatile float speed) {
  speedValueLeft = speed;
}

void carMazeOfOz::setSpeedRight(volatile float speed) {
  speedValueRight = speed;
}

float carMazeOfOz::getSpeedLeft() {
  return speedValueLeft;
}

float carMazeOfOz::getSpeedRight() {
  return speedValueRight;
}

float carMazeOfOz::getDistanceHead() {
  return distanceHead;
}

float carMazeOfOz::getDistanceLeft() {
  return distanceLeft;
}

float carMazeOfOz::getDistanceRight() {
  return distanceRight;
}

void carMazeOfOz::configureSpeed(volatile float &speedValueLeft, volatile float &speedValueRight) {
  static byte countStopLeft = 0, countStopRight = 0;
  static float lastSpeedValueLeft = speedValueLeft;
  static float lastSpeedValueRight = speedValueRight;
  if (lastSpeedValueLeft == speedValueLeft) countStopLeft++;
  else countStopLeft = 0;
  if (lastSpeedValueRight == speedValueRight) countStopRight++;
  else countStopRight = 0;
  if (countStopLeft > 10) speedValueLeft = 0;
  if (countStopRight > 10) speedValueRight = 0;
  lastSpeedValueLeft = speedValueLeft;
  lastSpeedValueRight = speedValueRight;
}
