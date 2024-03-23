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

enum MotorStep {
  MOTOR_1_STEP = 0,
  MOTOR_1_2_STEP,
  MOTOR_1_4_STEP,
  MOTOR_1_8_STEP,
  MOTOR_1_16_STEP,
  MOTOR_1_32_STEP,
  MOTOR_1_64_STEP,
  MOTOR_1_128_STEP,
  MOTOR_1_256_STEP,
  MOTOR_STEP_COUNT = 9
};

enum MotorRotateDirection {
  MOTOR_ROTATE_CW = 0,
  MOTOR_ROTATE_CCW,
  MOTOR_ROTATE_COUNT = 2
};

namespace interface {

class IDriver {
public:
  virtual ~IDriver() = default;

public:
  virtual void setStep(MotorStep step) = 0;
  virtual void setDirection(MotorRotateDirection direction) = 0;

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
