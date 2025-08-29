
#include "motor_ctrl.h"
#include "encoder_ctrl.h"

////// Pin declarations
static constexpr int8_t PIN_LED = 2;

//// Left wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_R_FORW = 18;
static constexpr int8_t PIN_R_BACK = 5;
// Encoder needs to support interrupts.
static constexpr int8_t PIN_R_ENCODER = 27;

//// Right wheel
// Control pins need to be able to generate PWM
static constexpr int8_t PIN_L_FORW = 21;
static constexpr int8_t PIN_L_BACK = 19;
// Encoder needs to support interrupts.
static constexpr int8_t PIN_L_ENCODER = 26;

////// Parameters of the robot
// Units in meters
static constexpr float WHEELS_Y_DISTANCE = 0.173;
static constexpr float WHEEL_RADIUS = 0.0245;
// Encoder value per revolution of left wheel and right wheel
static constexpr int TICK_PER_REVOLUTION = 4000;
// Min value for PWM that moves wheels
static constexpr int PWM_THRESHOLD = 100;

using namespace wheel_hal;

SinglePinEncoderCtrl left_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION, PIN_L_ENCODER, INPUT, FALLING);
SinglePinEncoderCtrl right_encoder(WHEEL_RADIUS, TICK_PER_REVOLUTION, PIN_R_ENCODER, INPUT, FALLING);

AT8236MotorCtrl left_motor(PIN_L_FORW, PIN_L_BACK, PWM_THRESHOLD);
AT8236MotorCtrl right_motor(PIN_R_FORW, PIN_R_BACK, PWM_THRESHOLD);

void setup()
{
  Serial.begin(115200);
  Serial.println("[INIT] Starting micro-ROS node...");

  left_motor.SetupPins();
  right_motor.SetupPins();

  left_encoder.SetupPins();
  right_encoder.SetupPins();
}

void loop()
{
  Serial.println("Left");
  auto left = left_encoder.GetEncoderMeasurement();
  Serial.println(left.delta_pos_m * 1000.0);
  Serial.println(left.delta_time_sec);

  Serial.println("Right");
  auto right = right_encoder.GetEncoderMeasurement();
  Serial.println(right.delta_pos_m * 1000.0);
  Serial.println(right.delta_time_sec);

  Serial.println();

  left_motor.SetSpeed(50, false);
  //right_motor.SetSpeed(50, true);
  delay(1000);

  left_motor.SetSpeed(0);
  right_motor.SetSpeed(0);
  delay(4000);
}
