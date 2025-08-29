# wheel-hal

Motor and Encoder Control Library for Arduino framework.

Tested on ESP32 hardware and also compile for Arduino Uno.

## Overview

`wheel-hal` provides a hardware abstraction layer for controlling motors and reading encoders. It includes:

- Abstract base classes for abstracting motor and encoder control
- Implementation for AT8236 motor driver
- Single-pin encoder support with hardware interrupts

## Features

- **Motor Control**: Set speed and direction, brake, and configure control pins
- **Encoder Reading**: Track wheel position and speed using hardware interrupts
- **Extensible**: Easily add support for other motor drivers or encoder types

## File Structure

- `src/motor_ctrl.h` / `src/motor_ctrl.cpp`: Motor control base and AT8236 implementation
- `src/encoder_ctrl.h` / `src/encoder_ctrl.cpp`: Encoder control base and single-pin implementation
- `examples/AT8236MotorCtrl/AT8236MotorCtrl.ino`: Example usage for a two-wheel robot

## API Reference

### Motor Control

- `BaseMotorCtrl`: Abstract interface
- `AT8236MotorCtrl`: Implements motor control for AT8236 driver
	- `SetupPins()`: Configure pins
	- `SetSpeed(percent, is_reverse)`: Set speed and direction
	- `Brake()`: Apply brake

### Encoder Control

- `BaseEncoderCtrl`: Abstract interface
- `SinglePinEncoderCtrl`: Implements single-pin encoder reading. **NOTE**: Currently only 4 instances of this class can be created since each maps to an explicitly declared interrupt handler.
	- `SetupPins()`: Configure encoder pin and interrupt
	- `GetEncoderMeasurement(is_in_reverse)`: Get position and time since last call
