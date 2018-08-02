
#include "PinChangeInterrupt.h"
#include <Wire.h>//Include the Wire library to talk I2C

//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0.
#define MCP4725_ADDR 0x60

#define X_PIN 5 // we could choose any pin
#define Y_PIN 6 // we could choose any pin

volatile int pwm_value_x = 0;
volatile int prev_time_x = 0;
volatile int pwm_value_y = 0;
volatile int prev_time_y = 0;


void change_x() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(X_PIN));
  if (trigger == RISING)
    prev_time_x = micros();
  else if (trigger == FALLING)
    pwm_value_x = micros() - prev_time_x;
  //  Serial.println(pwm_value_x);
}

void change_y() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(Y_PIN));
  if (trigger == RISING)
    prev_time_y = micros();
  else if (trigger == FALLING)
    pwm_value_y = micros() - prev_time_y;
  //  Serial.println(pwm_value_x);
}

void setup() {
  pinMode(X_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(X_PIN), change_x, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Y_PIN), change_y, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Wire.beginTransmission(MCP4725_ADDR);
  //  Wire.write(64);                     // cmd to update the DAC
  //  Wire.write(1);        // the 8 most significant bits...
  //  Wire.write(2); // the 4 least significant bits...
  //  Wire.endTransmission();
  Serial.print(pwm_value_x);
  Serial.print(", ");
  Serial.println(pwm_value_y);
}
