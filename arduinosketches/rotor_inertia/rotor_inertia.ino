#include <Arduino.h>
#include <Wire.h>
#include <math.h>

constexpr const int ESC_PWM_PORT = 2;
constexpr const int HALL_EFFECT_PINS[] = {3,4,5};

volatile long last_system_time = millis();
volatile double last_vel = 0;
long t0 = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ESC_PWM_PORT, OUTPUT);
  digitalWrite(ESC_PWM_PORT, 255);
  pinMode(HALL_EFFECT_PINS[0], INPUT);
  pinMode(HALL_EFFECT_PINS[1], INPUT);
  pinMode(HALL_EFFECT_PINS[2], INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PINS[0]), update_speed, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PINS[1]), update_speed, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PINS[2]), update_speed, RISING);
  t0 = millis();
}

void update_speed() {
  long new_time = millis();
  last_vel = (M_PI/3)/((new_time-last_system_time)/(double)1000); // delta rotation / delta time
  last_system_time = new_time;
}

void loop() {
  Serial.print("Time: ");
  Serial.print(millis() - t0);
  Serial.print(" velocity: ");
  Serial.println(last_vel);
}