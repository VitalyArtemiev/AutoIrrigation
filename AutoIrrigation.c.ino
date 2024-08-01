/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Z-Uno has an on-board LED you can control. It is attached to digital pin 13.
  LED_BUILTIN is an internal Define which can be used to access this LED.
  This example demonstrates the simple blink programm, where the blink interval can be changed from the Z-Wave controller.
  More information on http://z-uno.z-wave.me/

*/

#include "Arduino.h"
#include "ZUNO_Buttons.h"

#define LOOP_DELAY 1000
#define VALVE_DELAY 1000

#define TANK_AREA 1

#define MY_SERIAL Serial

#define NUM_PINS 26

#define ADC_BITS 12
#define ADC_COUNTS (1 << ADC_BITS)

#define BUTTON_VALVE_1_CLOSE 22
#define BUTTON_VALVE_1_OPEN 21
#define BUTTON_VALVE_2_CLOSE 20
#define BUTTON_VALVE_2_OPEN 19
#define BUTTON_RELAY_CLOSE 18
#define BUTTON_RELAY_OPEN 17

#define PIN_VALVE_1_OPEN 2
#define PIN_VALVE_1_CLOSE 24
#define PIN_VALVE_2_OPEN 0
#define PIN_VALVE_2_CLOSE 1
#define PIN_RELAY 8

#define PIN_PRESSURE A1

ZUNO_SETUP_DEBUG_MODE(DEBUG_ON);

word tank_water_volume = 0;
word pressure = 0;

byte valve_1 = 0;
byte valve_2 = 0;
byte relay = 0;

byte valve_1_actual = 0;
byte valve_2_actual = 0;
byte relay_actual = 0;

ZUNO_SETUP_CHANNELS(
  ZUNO_SENSOR_MULTILEVEL(ZUNO_SENSOR_MULTILEVEL_TYPE_TANK_CAPACITY, SENSOR_MULTILEVEL_SCALE_LITER, METER_SIZE_TWO_BYTES, SENSOR_MULTILEVEL_PRECISION_TWO_DECIMALS, tank_water_volume),
  ZUNO_SENSOR_MULTILEVEL(ZUNO_SENSOR_MULTILEVEL_TYPE_BAROMETRIC_PRESSURE, SENSOR_MULTILEVEL_SCALE_KILOPASCAL, METER_SIZE_TWO_BYTES, SENSOR_MULTILEVEL_PRECISION_TWO_DECIMALS, pressure),
  ZUNO_SWITCH_BINARY(valve_1, NULL),
  ZUNO_SWITCH_BINARY(valve_2, NULL),
  ZUNO_SWITCH_BINARY(relay, NULL),
  ZUNO_SENSOR_BINARY_WATER(valve_1_actual),
  ZUNO_SENSOR_BINARY_WATER(valve_2_actual),
  ZUNO_SENSOR_BINARY_WATER(relay_actual)
  );

#define CH_TANK 1
#define CH_PRESSURE 2
#define CH_VALVE_1 3
#define CH_VALVE_2 4
#define CH_RELAY 5
#define CH_VALVE_1_SENS 6
#define CH_VALVE_2_SENS 7
#define CH_RELAY_SENS 8

// the setup function runs once, when you press reset or power the board
void setup() {
  Serial.begin();
  // set digital pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  analogReadResolution(ADC_BITS);

  pinMode(BUTTON_VALVE_1_OPEN, INPUT_PULLUP);
  pinMode(BUTTON_VALVE_1_CLOSE, INPUT_PULLUP);
  pinMode(BUTTON_VALVE_2_OPEN, INPUT_PULLUP);
  pinMode(BUTTON_VALVE_2_CLOSE, INPUT_PULLUP);
  pinMode(BUTTON_RELAY_OPEN, INPUT_PULLUP);
  pinMode(BUTTON_RELAY_CLOSE, INPUT_PULLUP);

  pinMode(PIN_VALVE_1_OPEN, OUTPUT);
  pinMode(PIN_VALVE_1_CLOSE, OUTPUT);
  pinMode(PIN_VALVE_2_OPEN, OUTPUT);
  pinMode(PIN_VALVE_2_CLOSE, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);

  Serial.println("Setup complete");
}


bool write_rate_limited(dword* last_write, dword delay, int pin, int value) {
  int check = constrain(pin, 0, NUM_PINS);
  if (check != pin) {
    Serial.print("Incorrect pin requested. Ignoring.");
    return false;
  }

  dword now = millis();
  if (now - *last_write > delay) {
    digitalWrite(pin, value);
    *last_write = now;
    return true;
  }
  return false;
}

dword valve_semafore = millis();

void process_valves() {
  bool close;
  bool open;

  close = !digitalRead(BUTTON_VALVE_1_CLOSE);
  open = !digitalRead(BUTTON_VALVE_1_OPEN);
  byte valve_1_target = valve_1 && !close || open;

  if (valve_1_target != valve_1_actual) {
    Serial.println("Valve 1 wants to change to");
    Serial.println(valve_1_target);

    if (write_rate_limited(&valve_semafore, VALVE_DELAY, PIN_VALVE_1_OPEN, valve_1_target)) {
      digitalWrite(PIN_VALVE_1_CLOSE, !valve_1_target);
      valve_1_actual = valve_1_target;
      zunoSendReport(CH_VALVE_1_SENS);
      Serial.println("Done");
    }
  }

  close = !digitalRead(BUTTON_VALVE_2_CLOSE);
  open = !digitalRead(BUTTON_VALVE_2_OPEN);
  byte valve_2_target = valve_2 && !close || open;
  
  if (valve_2_target != valve_2_actual) {
    Serial.println("Valve 2 wants to change to");
    Serial.println(valve_2_target);

    if (write_rate_limited(&valve_semafore, VALVE_DELAY, PIN_VALVE_2_OPEN, valve_2_target)) {
      digitalWrite(PIN_VALVE_2_CLOSE, !valve_2_target);
      valve_2_actual = valve_2_target;
      zunoSendReport(CH_VALVE_2_SENS);
      Serial.println("Done");
    }  
  }
}

void process_relays() {
  byte old = relay_actual;

  bool close = !digitalRead(BUTTON_RELAY_CLOSE);
  bool open = !digitalRead(BUTTON_RELAY_OPEN);

  //Manual overrides
  relay_actual = relay && !close || open;

  if (old != relay_actual)
    zunoSendReport(CH_RELAY_SENS);
  // Low is on
  digitalWrite(PIN_RELAY, !relay_actual);
}

int adc_filter(int adc_pin, int cycles = 30) {
  cycles = constrain(cycles, 5, 100);
  int min = ADC_COUNTS;
  int max = 0;
  dword sum = 0;

  for (int i = 0; i < cycles; i++) {
    delay(10);
    int v = analogRead(adc_pin);

    if (v > max) {
      max = v;
    }
    if (v < min) {
      min = v;
    }
    sum += v;
  }
  return (sum - min - max) / (cycles - 2);
}

float get_voltage_naive(int adc_value) {
  float value = (float)adc_value;
  // Convert the analog reading (which goes from 0 - 4095) to a voltage (0 - 5V):
  float voltage = value * (5.0 / (float)ADC_COUNTS);
  return voltage;
}

float get_voltage_adjusted(int adc_value, int adc_reference) {
  float value = (float)adc_value;
  float reference = (float)adc_reference;
  if (reference == 0.0) {
    reference = 0.001;
  }
  // Convert the analog reading (which goes from 0 - 4095) to a voltage (0 - 5V):

  float voltage = value * (3.093 / reference);
  return voltage;
}

float voltage_to_pressure(float voltage) {
  // vcc = 5.0 vdc
  // Vout = Vcc(0.75 * P + 0.1)
  // P = (Vout / Vcc - 0.1) / 0.75 in MPa
  float pressure_kpa = (voltage / 4.8 - 0.095) / 0.75 * 1000;  // to kPa
  return pressure_kpa;
}

const int offset = 371;      // zero pressure adjust
const int fullScale = 3932;  // max pressure adjust

float get_pressure_naive(int adc_value) {
  float pressure_kpa = ((float)adc_value - offset) * 1200.0 / (fullScale - offset);
  return pressure_kpa;
}

float pressure_to_volume(float pressure_kpa, float area) {
  // p = ro * g * h

  float p = pressure_kpa * 1000;
  float h = p / 998 / 9.81;
  float v = h * area;
  return v;
}

void process_pressure() {
  // vcc = 5.0 vdc
  // Vout = Vcc(0.75 * P + 0.1)
  // P = (Vout / Vcc - 0.1) / 0.75
  // Vout = 3v / BATTERY * PIN_PRESSURE

  int reference = adc_filter(BATTERY);
  int pin_pressure = adc_filter(PIN_PRESSURE);

  float v = get_voltage_adjusted(pin_pressure, reference);

  word old_pressure = pressure;

  float pressure_kpa = voltage_to_pressure(v);

  // Two decimals precision
  pressure = (word)pressure_kpa * 100;
  tank_water_volume = (word)pressure_to_volume(pressure_kpa, TANK_AREA) * 100;

  if (abs(old_pressure - pressure) > 2 * 100) {
    zunoSendReport(CH_PRESSURE);
    zunoSendReport(CH_TANK);
  }

  // Serial.println("pressure");
  // Serial.println(pressure);

  // Serial.println("volume");
  // Serial.println(tank_water_volume);
}

// the loop function runs over and over again forever
void loop() {
  process_valves();
  process_relays();
  process_pressure();
  delay((dword)LOOP_DELAY - (millis() % LOOP_DELAY));
  Serial.println("valves");
  Serial.println(valve_1);
  Serial.println(valve_2);
  Serial.println(relay);

  Serial.println(valve_1_actual);
  Serial.println(valve_2_actual);
  Serial.println(relay_actual);
}
