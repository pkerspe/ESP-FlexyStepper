// Copyright 2024 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by jadjer on 3/21/24.
//

#pragma once

namespace motor::driver {

enum MotorRotateDirection {
  MOTOR_ROTATE_CW = 1,
  MOTOR_ROTATE_CCW = -1
};

enum MotorStep {
  MOTOR_FULL_STEP = 1,
  MOTOR_2_MICROSTEP = 2,
  MOTOR_4_MICROSTEP = 4,
  MOTOR_8_MICROSTEP = 8,
  MOTOR_16_MICROSTEP = 16,
  MOTOR_32_MICROSTEP = 32,
  MOTOR_64_MICROSTEP = 64,
  MOTOR_128_MICROSTEP = 128,
  MOTOR_256_MICROSTEP = 256,
  MOTOR_512_MICROSTEP = 512
};

namespace interface {

class IDriver {
public:
  virtual ~IDriver() = default;

public:
  [[nodiscard]] virtual uint32_t getMicrostep() const = 0;

public:
  virtual void setDirection(int8_t direction) = 0;
  virtual void setMicrostep(uint32_t microstep) = 0;

public:
  [[nodiscard]] virtual bool isEnabled() const = 0;
  [[nodiscard]] virtual bool isSleeping() const = 0;
  [[nodiscard]] virtual bool inHome() const = 0;
  [[nodiscard]] virtual bool isFault() const = 0;

public:
  virtual void enable() = 0;
  virtual void disable() = 0;

public:
  virtual void sleep() = 0;
  virtual void wake() = 0;

public:
  virtual void stepUp() = 0;
  virtual void stepDown() = 0;
};

}
}

#include <memory>

using IDriverPtr = std::unique_ptr<motor::driver::interface::IDriver>;
