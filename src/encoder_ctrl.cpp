#include "encoder_ctrl.h"

#include <math.h>

#include <Arduino.h>

using namespace wheel_hal;

// For ESP32 platforms IRQ code should be in IRAM.
#ifdef IRAM_ATTR
#define WHEEL_HAL_IRAM_ATTR IRAM_ATTR
#else
#define WHEEL_HAL_IRAM_ATTR
#endif

// While ESP32 code has FunctionalInterrupt.h, to support ATmega Arduino
// architectures, map each instance to its own interrupt handler.

typedef void (*voidFuncPtr)(void);

static constexpr size_t MAX_ENCODER_INSTANCES = 4;

static volatile unsigned long encoder_ticks[MAX_ENCODER_INSTANCES] = {0};

// Wrap singleton value in function to avoid initialization order issues.
static unsigned &GetStaticEncoderCount()
{
  static unsigned static_encoder_count = 0;
  return static_encoder_count;
}

static void handler0() { ++encoder_ticks[0]; }
static void handler1() { ++encoder_ticks[1]; }
static void handler2() { ++encoder_ticks[2]; }
static void handler3() { ++encoder_ticks[3]; }

static constexpr voidFuncPtr handlers[MAX_ENCODER_INSTANCES] = {handler0, handler1, handler2, handler3};

SinglePinEncoderCtrl::SinglePinEncoderCtrl(float wheel_radius_m,
                                           unsigned int ticks_per_rotation,
                                           uint8_t encoder_pin,
                                           uint8_t pin_mode,
                                           int irq_mode)
    : encoder_pin_(encoder_pin),
      pin_mode_(pin_mode),
      irq_mode_(irq_mode),
      ticks_to_m_ratio_(wheel_radius_m * 2.0 * M_PI / float(ticks_per_rotation)),
      // Capture the current encoder index and increment for the next instance.
      static_encoder_index_(GetStaticEncoderCount()++),
      last_poll_time_us_(micros())
{
}

SinglePinEncoderCtrl::~SinglePinEncoderCtrl()
{
  detachInterrupt(encoder_pin_);
}

void SinglePinEncoderCtrl::SetupPins()
{
  if (static_encoder_index_ >= MAX_ENCODER_INSTANCES)
  {
    // Add error reporting logic.
  }
  else
  {
    pinMode(this->encoder_pin_, this->pin_mode_);
    attachInterrupt(digitalPinToInterrupt(this->encoder_pin_),
                    handlers[static_encoder_index_],
                    this->irq_mode_);
  }
}

EncoderMeasurement SinglePinEncoderCtrl::GetEncoderMeasurement(bool is_in_reverse)
{
  if (static_encoder_index_ >= MAX_ENCODER_INSTANCES)
  {
    return EncoderMeasurement();
  }

  noInterrupts();
  unsigned local_encoder_ticks = encoder_ticks[static_encoder_index_];
  encoder_ticks[static_encoder_index_] = 0;
  interrupts();

  float delta_pos_m = float(local_encoder_ticks) * ticks_to_m_ratio_;
  if (is_in_reverse)
  {
    delta_pos_m = -delta_pos_m;
  }

  unsigned long now_us = micros();
  float delta_time_sec = float(now_us - last_poll_time_us_) / 1e6;
  last_poll_time_us_ = now_us;

  return EncoderMeasurement{delta_pos_m, delta_time_sec};
}

#undef WHEEL_HAL_IRAM_ATTR
