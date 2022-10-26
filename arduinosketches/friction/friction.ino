#include <Arduino.h>
#include <Wire.h>
#include <math.h>

constexpr const int ESC_PWM_PORT = 2;
constexpr const int HALL_EFFECT_PINS[] = {3,4,5};
constexpr const int CURRENT_SENSE_INPUT_PORT = 6;

const double CURRENT_SENSE_RESISTANCE = 0.1; 
constexpr const double PSU_VOLTAGE = 5;

// K_EMF should be calculated from the previous experiment
constexpr const double K_EMF = ((0.373 / 1000) / 60) * 2 * M_PI; // 0.373 mV/rpm * 1 V/1000 mv * 1 rps/60 rpm * 2pi rad/s / 1 rps

constexpr const int SPEEDS_TO_TEST[] = {10, 50, 100, 150, 200, 255};
constexpr const int NUM_PWM_SETTINGS = sizeof(SPEEDS_TO_TEST)/sizeof(int);
constexpr const long MILLI_PER_TEST = 2000;

volatile long last_system_time = millis();
volatile double last_vel = 0;

int pwm_index = 0;
double t0 = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ESC_PWM_PORT, OUTPUT);
  digitalWrite(ESC_PWM_PORT, SPEEDS_TO_TEST[pwm_index]);
  pinMode(HALL_EFFECT_PINS[0], INPUT);
  pinMode(HALL_EFFECT_PINS[1], INPUT);
  pinMode(HALL_EFFECT_PINS[2], INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PINS[0]), update_speed, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PINS[1]), update_speed, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_EFFECT_PINS[2]), update_speed, RISING);

  pinMode(CURRENT_SENSE_INPUT_PORT, INPUT);

  t0 = millis();
}

void update_speed() {
  long new_time = millis();
  last_vel = (M_PI/3)/((new_time-last_system_time)/(double)1000); // delta rotation / delta time
  last_system_time = new_time;
}

void loop() {
  const double V_sens = PSU_VOLTAGE * (double)analogRead(CURRENT_SENSE_INPUT_PORT) / 1024; // If motor is drawing 1 amp, voltage across resistor will be 0.1 V. Arduino ADC has resolution down to 5 mV.
  const double Ik_emf =  K_EMF * V_sens / CURRENT_SENSE_RESISTANCE;

  Serial.print("PWM: ");
  Serial.print(SPEEDS_TO_TEST[pwm_index]);
  Serial.print(" current: ");
  Serial.print(V_sens / CURRENT_SENSE_RESISTANCE);
  Serial.print(" omega: ");
  Serial.print(last_vel);
  Serial.print(" Ik_emf: ");
  Serial.println(Ik_emf);

  if (millis() - t0 >= MILLI_PER_TEST) {
    t0 = millis();
    pwm_index += 1;
    if (pwm_index = NUM_PWM_SETTINGS) {
      pwm_index = 0;
    }
    digitalWrite(ESC_PWM_PORT, SPEEDS_TO_TEST[pwm_index]);
  }
}