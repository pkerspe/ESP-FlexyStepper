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

// This library is based on the works of Paul Kerspe in his library:
// https://github.com/pkerspe/ESP-FlexyStepper

#pragma once

#include <cstdint>
#include <climits>
#include <functional>
#include <esp_task.h>

#include "motor/interface/ILimiter.hpp"
#include "motor/driver/interface/IDriver.hpp"

namespace motor {

using CallbackFunction = std::function<void()>;
using PositionCallbackFunction = std::function<void(long)>;

class MotorController {
public:
  explicit MotorController(IDriverPtr motorDriver);

public:
  void setHomeLimitSwitch(ILimiterPtr externalAction);

public:
  void registerHomeReachedCallback(CallbackFunction homeReachedCallbackFunction);
  void registerLimitReachedCallback(CallbackFunction limitSwitchTriggeredCallbackFunction);
  void registerTargetPositionReachedCallback(PositionCallbackFunction targetPositionReachedCallbackFunction);

public:
  void enableDriver();
  void disableDriver();

public:
  [[nodiscard]] bool isDriverEnabled() const;
  [[nodiscard]] bool isMotionComplete() const;
  [[nodiscard]] bool isMovingTowardsHome() const;

public:
  void clearLimitSwitchActive();
  [[nodiscard]] int8_t getDirectionOfMotion() const;

public:
  void setStepsPerMillimeter(float motorStepPerMillimeter);
  void setStepsPerRevolution(float motorStepPerRevolution);
  void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
  void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond);
  void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond);
  void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond);
  void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond);
  void setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond);
  void setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond);
  void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
  void setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond);
  void setDirectionToHome(int8_t directionTowardHome);
  void setLimitSwitchActive(int8_t limitSwitchType);

public:
  [[nodiscard]] float getCurrentVelocityInStepsPerSecond() const;
  [[nodiscard]] float getCurrentVelocityInRevolutionsPerSecond() const;
  [[nodiscard]] float getCurrentVelocityInMillimetersPerSecond() const;

public:
  [[nodiscard]] float getConfiguredAccelerationInStepsPerSecondPerSecond() const;
  [[nodiscard]] float getConfiguredAccelerationInRevolutionsPerSecondPerSecond() const;
  [[nodiscard]] float getConfiguredAccelerationInMillimetersPerSecondPerSecond() const;

public:
  [[nodiscard]] float getConfiguredDecelerationInStepsPerSecondPerSecond() const;
  [[nodiscard]] float getConfiguredDecelerationInRevolutionsPerSecondPerSecond() const;
  [[nodiscard]] float getConfiguredDecelerationInMillimetersPerSecondPerSecond() const;

public:
  void setCurrentPositionInSteps(long currentPositionInSteps);
  void setCurrentPositionInMillimeters(float currentPositionInMillimeters);
  void setCurrentPositionInRevolutions(float currentPositionInRevolutions);

public:
  [[nodiscard]] long getCurrentPositionInSteps() const;
  [[nodiscard]] float getCurrentPositionInRevolutions() const;
  /**
   * @return position in millimeters
   */
  [[nodiscard]] float getCurrentPositionInMillimeters() const;

public:
  void startJogging(int8_t direction);
  void stopJogging();
  void goToLimitAndSetAsHome(CallbackFunction const &callbackFunctionForHome = nullptr, long maxDistanceToMoveInSteps = 2000000000L);
  void goToLimit(int8_t direction, CallbackFunction const &callbackFunctionForLimit = nullptr);

public:
  void setCurrentPositionAsHomeAndStop();
  void setTargetPositionToStop();
  [[nodiscard]] int32_t getDistanceToTargetSigned() const;

public:
  void setTargetPositionInSteps(long absolutePositionToMoveToInSteps);
  void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  void setTargetPositionRelativeInSteps(long distanceToMoveInSteps);
  void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters);
  void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions);

public:
  [[nodiscard]] int32_t getTargetPositionInSteps() const;
  [[nodiscard]] float getTargetPositionInMillimeters() const;
  [[nodiscard]] float getTargetPositionInRevolutions() const;

public:
  void moveToPositionInSteps(long absolutePositionToMoveToInSteps);
  void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  void moveRelativeInSteps(long distanceToMoveInSteps);
  void moveRelativeInMillimeters(float distanceToMoveInMillimeters);
  void moveRelativeInRevolutions(float distanceToMoveInRevolutions);

public:
  bool moveToHomeInSteps(int8_t directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps);
  bool moveToHomeInMillimeters(int8_t directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters);
  bool moveToHomeInRevolutions(int8_t directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions);

public:
  // the central function to calculate the next movment step signal
  bool processMovement();

private:
  static void taskRunner(void *parameter);

private:
  void determinePeriodOfNextStep();

private:
  static const int8_t LIMIT_SWITCH_BEGIN = -1;
  static const int8_t LIMIT_SWITCH_END = 1;
  static const int8_t LIMIT_SWITCH_COMBINED_BEGIN_AND_END = 2;

private:
  IDriverPtr m_motorDriver = nullptr;
  ILimiterPtr m_homeLimitSwitch = nullptr;

private:
  CallbackFunction m_homeReachedCallback = nullptr;
  CallbackFunction m_limitTriggeredCallback = nullptr;
  CallbackFunction m_callbackFunctionForGoToLimit = nullptr;
  PositionCallbackFunction m_targetPositionReachedCallback = nullptr;

private:
  int8_t m_activeLimitSwitch;
  int8_t m_directionOfMotion;
  int8_t m_disallowedDirection;
  int8_t m_directionTowardsHome;
  int8_t m_lastStepDirectionBeforeLimitSwitchTrigger;

private:
  bool m_isOnWayToHome = false;
  bool m_isOnWayToLimit = false;
  bool m_isCurrentlyHomed;
  bool m_limitSwitchCheckPerformed;
  bool m_firstProcessingAfterTargetReached = true;

private:
  int32_t m_targetPosition_InSteps;
  int32_t m_currentPosition_InSteps;

private:
  uint32_t m_lastStepTime_InUS;

private:
  float m_stepsPerMillimeter;
  float m_stepsPerRevolution;
  float m_nextStepPeriod_InUS;
  float m_currentStepPeriod_InUS;
  float m_periodOfSlowestStep_InUS;
  float m_desiredPeriod_InUSPerStep;
  float m_desiredSpeed_InStepsPerSecond;
  float m_acceleration_InStepsPerUSPerUS;
  float m_deceleration_InStepsPerUSPerUS;
  float m_minimumPeriodForAStoppedMotion;
  float m_acceleration_InStepsPerSecondPerSecond;
  float m_deceleration_InStepsPerSecondPerSecond;
};

}
