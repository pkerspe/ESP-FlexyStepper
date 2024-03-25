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

#include <vector>
#include <cstdint>
#include <functional>

#include "motor/interface/ILimiter.hpp"
#include "motor/driver/interface/IDriver.hpp"

/**
 * @namespace motor
 * @brief Motor namespace
 */
namespace motor {

using Limiters = std::vector<ILimiterPtr>;
using CallbackFunction = std::function<void()>;
using PositionCallbackFunction = std::function<void(int32_t)>;

/**
 * @class MotorController
 * @brief Class for controlling a stepper motor
 */
class MotorController {
public:
  explicit MotorController(IDriverPtr motorDriver, ILimiterPtr beginLimiter = nullptr, ILimiterPtr endLimiter = nullptr);

public:
  /**
   * @brief Set a callback function that will be called when the motor reaches the home position
   * @param homeReachedCallbackFunction
   */
  void registerHomeReachedCallback(CallbackFunction homeReachedCallbackFunction);
  /**
   * @brief Set a callback function that will be called when the motor reaches the limit switch
   * @param limitSwitchTriggeredCallbackFunction
   */
  void registerLimitReachedCallback(CallbackFunction limitSwitchTriggeredCallbackFunction);
  /**
   * @brief Set a callback function that will be called when the motor reaches the target position
   * @param targetPositionReachedCallbackFunction
   */
  void registerTargetPositionReachedCallback(PositionCallbackFunction targetPositionReachedCallbackFunction);

public:
  /**
   * return true if homed
   * @return
   */
  [[nodiscard]] bool isHomed() const;
  /**
   * return true if driver is enabled
   * @return
   */
  [[nodiscard]] bool isDriverEnabled() const;
  /**
   * return true if motion is complete
   * @return
   */
  [[nodiscard]] bool isMotionComplete() const;

public:
  /**
   * get direction of motion
   * @return
   */
  [[nodiscard]] int8_t getDirectionOfMotion() const;

public:
  /**
   * get target position in steps
   * @return
   */
  [[nodiscard]] int32_t getTargetPositionInSteps() const;
  /**
   * get current position in steps
   * @return
   */
  [[nodiscard]] int32_t getCurrentPositionInSteps() const;
  /**
   * get distance to target
   * @return
   */
  [[nodiscard]] int32_t getDistanceToTargetSigned() const;

public:
  /**
   * get target position in millimeters
   * @return
   */
  [[nodiscard]] float getTargetPositionInMillimeters() const;
  /**
   * get target position in revolutions
   * @return
   */
  [[nodiscard]] float getTargetPositionInRevolutions() const;

public:
  /**
   * get current position in revolutions
   * @return
   */
  [[nodiscard]] float getCurrentPositionInMillimeters() const;
  /**
   * get current position in millimeters
   * @return
   */
  [[nodiscard]] float getCurrentPositionInRevolutions() const;

public:
  /**
   * get current velocity in steps per second
   * @return
   */
  [[nodiscard]] float getCurrentVelocityInStepsPerSecond() const;
  /**
   * get current velocity in millimeters per second
   * @return
   */
  [[nodiscard]] float getCurrentVelocityInMillimetersPerSecond() const;
  /**
   * get current velocity in revolutions per second
   * @return
   */
  [[nodiscard]] float getCurrentVelocityInRevolutionsPerSecond() const;

public:
  /**
   * get configured acceleration in steps per second per second
   * @return
   */
  [[nodiscard]] float getConfiguredAccelerationInStepsPerSecondPerSecond() const;
  /**
   * get configured acceleration in millimeters per second per second
   * @return
   */
  [[nodiscard]] float getConfiguredAccelerationInMillimetersPerSecondPerSecond() const;
  /**
   * get configured acceleration in revolutions per second per second
   * @return
   */
  [[nodiscard]] float getConfiguredAccelerationInRevolutionsPerSecondPerSecond() const;

public:
  /**
   * get configured deceleration in steps per second per second
   * @return
   */
  [[nodiscard]] float getConfiguredDecelerationInStepsPerSecondPerSecond() const;
  /**
   * get configured deceleration in millimeters per second per second
   * @return
   */
  [[nodiscard]] float getConfiguredDecelerationInMillimetersPerSecondPerSecond() const;
  /**
   * get configured deceleration in revolutions per second per second
   * @return
   */
  [[nodiscard]] float getConfiguredDecelerationInRevolutionsPerSecondPerSecond() const;

public:
  /**
   * set steps per millimeter
   * @param motorStepPerMillimeter
   */
  void setStepsPerMillimeter(float motorStepPerMillimeter);
  /**
   * set steps per revolution
   * @param motorStepPerRevolution
   */
  void setStepsPerRevolution(float motorStepPerRevolution);

public:
  /**
   * set speed in steps per second
   * @param speedInStepsPerSecond
   */
  void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
  /**
   * set speed in revolutions per second
   * @param speedInRevolutionsPerSecond
   */
  void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond);
  /**
   * set speed in millimeters per second
   * @param speedInMillimetersPerSecond
   */
  void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond);

public:
  /**
   * set acceleration in steps per second per second
   * @param accelerationInStepsPerSecondPerSecond
   */
  void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
  /**
   * set acceleration in millimeters per second per second
   * @param accelerationInMillimetersPerSecondPerSecond
   */
  void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond);
  /**
   * set acceleration in revolutions per second per second
   * @param accelerationInRevolutionsPerSecondPerSecond
   */
  void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond);

public:
  /**
   * set deceleration in steps per second per second
   * @param decelerationInStepsPerSecondPerSecond
   */
  void setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond);
  /**
   * set deceleration in millimeters per second per second
   * @param decelerationInMillimetersPerSecondPerSecond
   */
  void setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond);
  /**
   * set deceleration in revolutions per second per second
   * @param decelerationInRevolutionsPerSecondPerSecond
   */
  void setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond);

public:
  /**
   * set current position in steps
   * @param currentPositionInSteps
   */
  void setCurrentPositionInSteps(int32_t currentPositionInSteps);
  /**
   * set current position in millimeters
   * @param currentPositionInMillimeters
   */
  void setCurrentPositionInMillimeters(float currentPositionInMillimeters);
  /**
   * set current position in revolutions
   * @param currentPositionInRevolutions
   */
  void setCurrentPositionInRevolutions(float currentPositionInRevolutions);
  /**
   * set current position as home and stop
   */
  void setCurrentPositionAsHomeAndStop();

public:
  /**
   * set target position in steps
   * @param absolutePositionToMoveToInSteps
   */
  void setTargetPositionToStop();
  /**
   * set target position in steps
   * @param absolutePositionToMoveToInSteps
   */
  void setTargetPositionInSteps(int32_t absolutePositionToMoveToInSteps);
  /**
   * set target position in millimeters
   * @param absolutePositionToMoveToInMillimeters
   */
  void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  /**
   * set target position in revolutions
   * @param absolutePositionToMoveToInRevolutions
   */
  void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  /**
   * set target position relative to current position in steps
   * @param distanceToMoveInSteps
   */
  void setTargetPositionRelativeInSteps(int32_t distanceToMoveInSteps);
  /**
   * set target position relative to current position in millimeters
   * @param distanceToMoveInMillimeters
   */
  void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters);
  /**
   * set target position relative to current position in revolutions
   * @param distanceToMoveInRevolutions
   */
  void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions);

public:
  /**
   * enable driver
   */
  void enableDriver();
  /**
   * disable driver
   */
  void disableDriver();

public:
  /**
   * start jogging
   * @param direction
   */
  void startJogging(int8_t direction);
  /**
   * stop jogging
   */
  void stopJogging();

public:
  /**
   * Go to begin limit.
   * Blocked until motion is complete.
   */
  void goToLimit(int8_t direction, float speedInStepsPerSecond);
  /**
   * Go to end limit and set as home.
   * Blocked until motion is complete.
   */
  void goToLimitAndSetAsHome(int8_t direction, float speedInStepsPerSecond);

public:
  /**
   * Move to position in steps.
   * Blocked until motion is complete.
   * @param absolutePositionToMoveToInSteps
   */
  void moveToPositionInSteps(int32_t absolutePositionToMoveToInSteps);
  /**
   * Move to position in millimeters.
   * Blocked until motion is complete.
   * @param absolutePositionToMoveToInMillimeters
   */
  void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  /**
   * Move to position in revolutions.
   * Blocked until motion is complete.
   * @param absolutePositionToMoveToInRevolutions
   */
  void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  /**
   * Move to position relative to current position in steps. Blocked until motion is complete.
   * Blocked until motion is complete.
   * @param distanceToMoveInSteps
   */
  void moveRelativeInSteps(int32_t distanceToMoveInSteps);
  /**
   * Move to position relative to current position in millimeters.
   * Blocked until motion is complete.
   * @param distanceToMoveInMillimeters
   */
  void moveRelativeInMillimeters(float distanceToMoveInMillimeters);
  /**
   * Move to position relative to current position in revolutions
   * Blocked until motion is complete.
   * @param distanceToMoveInRevolutions
   */
  void moveRelativeInRevolutions(float distanceToMoveInRevolutions);
  /**
   * Move to home
   * Blocked until motion is complete.
   * @param speedInStepsPerSecond
   * @return
   */
  void moveToHome();

public:
  /**
   * Process movement.
   * Return true if movement is not complete.
   * @return
   */
  bool processMovement();

private:
  Limiters m_limiters;
  IDriverPtr m_motorDriver;

private:
  CallbackFunction m_homeReachedCallback;
  CallbackFunction m_limitTriggeredCallback;
  CallbackFunction m_callbackFunctionForGoToLimit;
  PositionCallbackFunction m_targetPositionReachedCallback;

private:
  int8_t m_directionOfMotion;
  int8_t m_disallowedDirection;

private:
  bool m_isOnWayToHome;
  bool m_limitSwitchCheckPerformed;
  bool m_firstProcessingAfterTargetReached;

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

private:
  /**
   * Return true if movement is any limit switch triggered
   * @return
   */
  [[nodiscard]] bool checkingActivityOfLimiters();

private:
  /**
   * Determine period of next step
   */
  void determinePeriodOfNextStep();
};

}

#include <memory>

using MotorControllerPtr = std::unique_ptr<motor::MotorController>;
