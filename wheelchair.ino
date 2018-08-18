
#include "PinChangeInterrupt.h"
#include <Adafruit_MCP4725.h>

//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0.
#define MCP4725_ADDR_X 0x61
#define MCP4725_ADDR_Y 0x60

Adafruit_MCP4725 dac_x; // leftright
Adafruit_MCP4725 dac_y; // forwardaft

#define X_PIN 5 // we could choose any pin
#define Y_PIN 6 // we could choose any pin
#define SWITCH_PIN 3 // we could choose any pin

#define JOY_X_1 A0
#define JOY_X_2 A1
#define JOY_Y_1 A2
#define JOY_Y_2 A3
#define JOY_V2 A4

#define MAXIMUM_VOLTAGE 3.5
#define VOLTAGE_SWING 1.5 // measured from joystick 2.07
#define REFERENCE_VOLTAGE 5
#define MIDDLE_VOLTAGE 2.54 // measured from controller

volatile int pwm_value_x = 0;
volatile int prev_time_x = 0;
volatile int pwm_value_y = 0;
volatile int prev_time_y = 0;
volatile int pwm_value_switch = 0;
volatile int prev_time_switch = 0;
int output_x = 0, output_y = 0, minimum_output = 0, maximum_output = 4095, joystick_output_x = 0, joystick_output_y = 0;
int max_x = 0, min_x = 2000, max_y = 0, min_y = 2000;
bool joystick_in_control = false;

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

void change_switch() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(SWITCH_PIN));
  if (trigger == RISING)
    prev_time_switch = micros();
  else if (trigger == FALLING)
    pwm_value_switch = micros() - prev_time_switch;
  //  Serial.println(pwm_value_x);
}


void setup() {
  pinMode(X_PIN, INPUT_PULLUP);
  pinMode(Y_PIN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(X_PIN), change_x, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Y_PIN), change_y, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SWITCH_PIN), change_switch, CHANGE);
  dac_x.begin(MCP4725_ADDR_X); // The I2C Address: Run the I2C Scanner if you're not sure
  dac_y.begin(MCP4725_ADDR_Y);
  minimum_output = (MIDDLE_VOLTAGE - VOLTAGE_SWING) * 4096 / REFERENCE_VOLTAGE;
  maximum_output = (MIDDLE_VOLTAGE + VOLTAGE_SWING) * 4096 / REFERENCE_VOLTAGE;

}


void loop() {
  joystick_output_x = map(analogRead(JOY_X_1), 0, 1023, 0, 4095);
  joystick_output_y = map(analogRead(JOY_Y_1), 0, 1023, 0, 4095);
  if (pwm_value_switch < 1500) {
    joystick_in_control = true;
  } else {
    joystick_in_control = false;
  }
  // Big joystick movement overrides RC interface
  if (abs(joystick_output_x - 2000) > 500 || abs(joystick_output_y - 2000) > 500) {
    joystick_in_control = true;
  }
  // V2 should be around half the voltage, i.e. 2.5 V = 512. A large value indicates that the joystick is not connected.
  if (analogRead(JOY_V2) > 900) {
    joystick_in_control = false;
  }

  if (pwm_value_x > 800 && pwm_value_y > 800) {
    max_x = max(max_x, pwm_value_x);
    min_x = min(min_x, pwm_value_x);
    max_y = max(max_y, pwm_value_y);
    min_y = min(min_y, pwm_value_y);


    output_x = map(pwm_value_x, min_x, max_x, minimum_output, maximum_output);
    output_y = map(pwm_value_y, min_y, max_y, minimum_output, maximum_output);
    output_x = min(output_x, maximum_output);
    output_x = max(output_x, minimum_output);
    output_y = min(output_y, maximum_output);
    output_y = max(output_y, minimum_output);
  }
  if (joystick_in_control) {
    dac_x.setVoltage(joystick_output_x, false);
    dac_y.setVoltage(joystick_output_y, false);
  } else {
    dac_x.setVoltage(output_x, false);
    dac_y.setVoltage(output_y, false);
  }
  /*
    Serial.print("Switch: ");
    Serial.println(pwm_value_switch);
    Serial.print(" JOY_V2: ");
    Serial.print(analogRead(JOY_V2));
  */
  /*
    Serial.print(pwm_value_x);
    Serial.print(", ");
    Serial.print(output_x);
    Serial.print(", ");
    Serial.print(pwm_value_y);
    Serial.print(", ");
    Serial.println(output_y);
  */
  Serial.print("Switch: ");
  Serial.print(pwm_value_switch);

  Serial.print(" JOY_X_1: ");
  Serial.print(analogRead(JOY_X_1));
  Serial.print(" JOY_X_2: ");
  Serial.print(analogRead(JOY_X_2));
  Serial.print(" JOY_Y_1: ");
  Serial.print(analogRead(JOY_Y_1));
  Serial.print(" JOY_Y_2: ");
  Serial.print(analogRead(JOY_Y_2));
  Serial.print(" JOY_V2: ");
  Serial.println(analogRead(JOY_V2));
  
}
