//
//   Good Fan Control 0001
//
// Author: Vincent Brubaker-Gianakos
// Date: 2020
// License: MIT

#include <SoftwareSerial.h>
#include <AceRoutine.h>

#define USE_FAST_PWM

#ifdef USE_FAST_PWM
#include <PWM.h>
#endif

using namespace ace_routine;

// Settings
// Interpolant used for smoothing input to output
float Feathering = 0.02;

// Minimum allowed valid target value (used to prevent motor stalling)
float MinTargetValue = 0.10;

// Minimum starting torque (inertia) compensation value
float TorqueCompMinValue = 0.70;

// Maximum delta between input value and validated target value
float DeltaWarningTolerance = 0.0001;

// Time in milliseconds between value processing
int ProcessValuesPeriod = 20;

// Pinouts
// Software serial connection to SparkFun 7-segment 4 digit LED readout display
SoftwareSerial Serial7Segment(2, 13); // RX pin, TX pin

// Analog pin number used for delta warning indicator
const int DELTA_WARNING_INDICATOR_PIN_NUMBER = 3;

// Motor controller pins
const int
  MOTOR_CONTROLLER_ENA_PIN_NUMBER = 6,
  MOTOR_CONTROLLER_ENB_PIN_NUMBER = 10,
  MOTOR_CONTROLLER_IN1_PIN_NUMBER = 7,
  MOTOR_CONTROLLER_IN2_PIN_NUMBER = 8,
  MOTOR_CONTROLLER_IN3_PIN_NUMBER = 9,
  MOTOR_CONTROLLER_IN4_PIN_NUMBER = 11;

// Input parameters
float TargetValue = 0.0;

// Output parameters
float PriorTorqueValue = 0.0, PriorValue = 0.0, OutputValue = 0.0;

// Indicates a disparity between input value and validated target value
int DeltaWarning = 0;

// Constants used for display
const byte DISPLAY_DEFAULT_BRIGHTNESS_LEVEL = 0;
const byte DISPLAY_MAX_BRIGHTNESS_LEVEL_LIMIT = 100;

// More constants
const int INDICATOR_PIN_STANDARD_WRITE_LEVEL = 2;
const int INDICATOR_PIN_SAFETY_WRITE_LEVEL = 64;

enum IndicatorState {
  INDICATOR_OFF = 0,
  INDICATOR_ON = 1,
  INDICATOR_SAFETY_CHECK = 2
};

// Sets up the indicator pins (output mode)
void setup_indicator_pin_mode()
{
  pinMode(DELTA_WARNING_INDICATOR_PIN_NUMBER, OUTPUT);
}

// Sets up the pins for the motor controller(s) (output mode)
void setup_motor_controller_pin_mode()
{
  pinMode(MOTOR_CONTROLLER_ENA_PIN_NUMBER, OUTPUT);
  pinMode(MOTOR_CONTROLLER_ENB_PIN_NUMBER, OUTPUT);
  pinMode(MOTOR_CONTROLLER_IN1_PIN_NUMBER, OUTPUT);
  pinMode(MOTOR_CONTROLLER_IN2_PIN_NUMBER, OUTPUT);
  pinMode(MOTOR_CONTROLLER_IN3_PIN_NUMBER, OUTPUT);
  pinMode(MOTOR_CONTROLLER_IN4_PIN_NUMBER, OUTPUT);
}

// Run the safety check (for indicators)
// Momentarily set all indicator pins to INDICATOR_SAFETY_CHECK state
void setup_safety_check()
{
  set_indicator_pin(DELTA_WARNING_INDICATOR_PIN_NUMBER, INDICATOR_SAFETY_CHECK);
  delay(1000);
  set_indicator_pin(DELTA_WARNING_INDICATOR_PIN_NUMBER, INDICATOR_OFF);
}

// Clears the message on the display connected by software serial
void reset_display(SoftwareSerial& serial)
{
  serial.write(0x76);
}

// Sets the message on a 4-digit display connected by software serial
void set_display4_message(SoftwareSerial& serial, const char* message)
{
  for (int i = 0; i < 4; i++)
  {
    char c = message[i];
    if (!c)
      break;
    serial.write(message[i]);
  }
}

// Formats a float value with a given precision for a 4-digit display with decimal LEDs
// Produces digits_out and decimal_place_out
void display4_dtostrf(char* digits_out, int& decimal_place_out, int precision, float value)
{
  // 5 digits (including decimal) + \0 (null)
  char value_str[6];

  //sprintf(formatted, "%5.*f", precision, value);
  dtostrf(value, 5, precision, value_str);

  int digit_count = 0;
  decimal_place_out = 3;
  for (int i = 0; i < 5 && digit_count < 4; i++)
  {
    char c = value_str[i];
    switch (c)
    {
      default:
        digits_out[digit_count] = c;
        digit_count++;
        break;
      case '.':
        if (digit_count > 0)
          decimal_place_out = digit_count - 1;
    }
  }
}

// Sets the message on a 4-digit display connected by software serial to a formatted floating point value
void set_display4_float(SoftwareSerial& serial, int precision, float value)
{
  char digits[4];
  int decimal_place;
  display4_dtostrf(digits, decimal_place, precision, value);
  reset_display(serial);
  serial.write(0x77);
  serial.write(1 << decimal_place);
  for (int i = 0; i < 4; i++)
    serial.write(digits[i]);
}

// Sets the brightness of the display connected by software serial
void set_display_brightness_level(SoftwareSerial& serial, const byte brightness_level)
{
  // Brightness control command
  serial.write(0x7A);
  byte brightness_level_limited = min(brightness_level, DISPLAY_MAX_BRIGHTNESS_LEVEL_LIMIT);
  // Brightness level is inverted for some reason?
  serial.write(100 - brightness_level_limited);
}

// Sets the state of an indicator pin
void set_indicator_pin(const int pin_number, IndicatorState state)
{
  int write_level;
  switch (state)
  {
    default:
      write_level = 0;
      break;
    case INDICATOR_ON:
      write_level = INDICATOR_PIN_STANDARD_WRITE_LEVEL;
      break;
    case INDICATOR_SAFETY_CHECK:
      write_level = INDICATOR_PIN_SAFETY_WRITE_LEVEL;
      break;
  }
  analogWrite(pin_number, write_level);
}

#ifdef USE_FAST_PWM
void _set_fast_pwm_auto(const uint8_t &timer, const char &abcd, const uint32_t &frequency_hz, const float &duty_cycle)
{
  if (duty_cycle <= 0.0001)
    pwm.set(timer, abcd, frequency_hz, 1, true);
  else if (duty_cycle >= 0.9999)
    pwm.set(timer, abcd, frequency_hz, 1, false);
  else
  {
    uint16_t divisor;
    float t;
    bool invert;
    if (duty_cycle < 0.5)
    {
      t = duty_cycle;
      invert = false;
    }
    else
    {
      t = 1.0 - duty_cycle;
      invert = true;
    }
    divisor = min(lround(1.0 / t), 0xFFFF);
    pwm.set(timer, abcd, frequency_hz, divisor, invert);
  }
}
#endif

// Sets the a motor controller's (signed) speed and direction given
// an enable pin number for PWM and two pins for directional control
void _set_motor(const int enable_pin, const int dir_0_pin, const int dir_1_pin, float speed)
{
  float speed_clamped = min(max(-1.0, speed), 1.0);
  int speed_int = lround(255 * speed_clamped);
#ifdef USE_FAST_PWM
  switch (enable_pin)
  {
    case 5:
      _set_fast_pwm_auto(3, 'a', 200000, speed_clamped);
      break;
    case 6:
      _set_fast_pwm_auto(4, 'd', 1200000, speed_clamped);
    case 10:
      _set_fast_pwm_auto(4, 'b', 1200000, speed_clamped);
      break;
    default:
      analogWrite(enable_pin, labs(speed_int));
      break;
  }
#else
  analogWrite(enable_pin, labs(speed_int));
#endif

  digitalWrite(dir_0_pin, speed_int > 0 ? HIGH : LOW);
  digitalWrite(dir_1_pin, speed_int < 0 ? HIGH : LOW);
}

// Set (direction-signed) speed for motor controller A
void set_motor_a(float speed)
{
  _set_motor(
    MOTOR_CONTROLLER_ENA_PIN_NUMBER,
    MOTOR_CONTROLLER_IN1_PIN_NUMBER,
    MOTOR_CONTROLLER_IN2_PIN_NUMBER,
    speed);
}

// Set (direction-signed) speed for motor controller B
void set_motor_b(float speed)
{
  _set_motor(
    MOTOR_CONTROLLER_ENB_PIN_NUMBER,
    MOTOR_CONTROLLER_IN3_PIN_NUMBER,
    MOTOR_CONTROLLER_IN4_PIN_NUMBER,
    speed);
}

// Constrain the target value
// Returns 0 for values less than or equal to 0
// Returns minimum value for value less than min_target_value
float validate_target_value(const float min_target_value, float value)
{
  if (value > 0.0)
    return min(max(min_target_value, value), 1.0);
  return 0.0;
}

// Constrained linear interpolation between a and b by interpolant t
float lerpf(const float a, const float b, const float t)
{
  return a + (b - a) * min(max(0.0, t), 1.0);
}

// Rounds a float to a given number of decimal places
// For negative precision values, this will round to 10s, 100s, 1000s, etc.
float round_decimal(const int precision, float value)
{
  const float scale = pow(10.0, (float)precision);
  return round(scale * value) / scale;
}

// Parses/processes input stream from hardware serial connection
COROUTINE(ProcessSerial)
{
  COROUTINE_LOOP() {
    if (Serial.available())
    {
      TargetValue = Serial.parseFloat() / 100.0;
      // TargetValue is rounded to 3 decimal places so that there is
      // no disparity between the number displayed and conditions
      // such as delta warning
      TargetValue = round_decimal(3, TargetValue);
      while (Serial.available())
        Serial.read();
    }
    COROUTINE_DELAY(50);
  }
}

// Processes the (input) target values and produces conditioned output values
COROUTINE(ProcessValues)
{
  COROUTINE_LOOP() {
    // Validated form of TargetValue
    float target_value_v = validate_target_value(MinTargetValue, TargetValue);

    // This is calibrated
    float torque = (1.0 - pow(1.0 - max(PriorValue, target_value_v), 16));

    // Feathered torque value
    PriorTorqueValue = lerpf(PriorTorqueValue, torque, Feathering);

    // Torque compensation
    // Rectified + clipped + HPF
    float torque_comp = min(max(0.0, torque - PriorTorqueValue), 1.0);

    // Torque power: used for lag compensation based on the given filter model
    // (feathering based on iterative linear interpolation)
    //
    // An incremental linear interpolation from a prior value (a) toward a target value (b)
    // with a feathering value (t) by a number of steps (n) can be modeled as
    //
    //    a' = a + (b - a) * t
    // or
    //    a' = a * (1 - t) + b * t
    //
    // To solve for the number of steps (n) to reach a given crossover point (c)
    // between normalized values a_0 = 1, b = 0, where c = (0,1)
    //
    //    a' = a * (1 - t)
    //    a_n = (1 - t)^n  // Product expansion
    //    c = a_n = (1 - t)^n
    //    n = log(c) / log(1 - t)
    //
    // Q.E.D
    const float crossover = 0.5;

    // Depends on the degree (filter order) of output feathering
    const float torque_power_coef = 2.0;

    // Calculate torque power (exponent) multiplier based on crossover and feathering
    float torque_power = torque_power_coef * pow(1.0 + Feathering, log(crossover) / log(1.0 - Feathering));

    // Scale the torque compensation by the minumum amount
    float torque_scale = TorqueCompMinValue * torque_power;

    // Use validated target value or torque compensation, whichever is larger
    float max_target = max(target_value_v, torque_scale * torque_comp);

    // Feathered prior value
    PriorValue = lerpf(PriorValue, max_target, Feathering);

    // Output is double-feathered
    OutputValue = lerpf(OutputValue, PriorValue, Feathering);

    // Delta warning means that the input TargetValue differs significantly from its validated form target_value
    DeltaWarning = abs(target_value_v - TargetValue) > DeltaWarningTolerance ? 1 : 0;

    // Debugging
    /*
    Serial.print(100.0 * TargetValue);
    Serial.print(" ");
    Serial.print(100.0 * PriorValue);
    Serial.print(" ");
    Serial.print(100.0 * OutputValue);
    Serial.println();
    */

    COROUTINE_DELAY(ProcessValuesPeriod);
  }
}

// Periodically updates the display and indicators to show the target value as a percentage
COROUTINE(UpdateDisplay)
{
  COROUTINE_LOOP() {
    // Display TargetValue as a percentage
    set_display4_float(Serial7Segment, 1, 100.0 * TargetValue);
    // Set indicators
    set_indicator_pin(DELTA_WARNING_INDICATOR_PIN_NUMBER, DeltaWarning ? INDICATOR_ON : INDICATOR_OFF);
    COROUTINE_DELAY(20);
  }
}

// Periodically update the motor controller to set the output value
COROUTINE(UpdateMotorController)
{
  COROUTINE_LOOP() {
    set_motor_a(OutputValue);
    set_motor_b(OutputValue);
    COROUTINE_DELAY(50);
  }
}

void setup()
{
  // Indicators setup
  setup_indicator_pin_mode();
  setup_safety_check();

  setup_motor_controller_pin_mode();

  // Begin serial
  Serial.begin(9600);
  Serial7Segment.begin(9600);

  // Display setup
  reset_display(Serial7Segment);
  set_display_brightness_level(Serial7Segment, DISPLAY_DEFAULT_BRIGHTNESS_LEVEL);

  // Coroutine setup
  CoroutineScheduler::setup();
}

void loop()
{
  CoroutineScheduler::loop();
}
