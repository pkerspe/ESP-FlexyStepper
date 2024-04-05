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

#include <cmath>
#include <utility>
#include <thread>
#include <esp_timer.h>

#include "motor/MotorController.hpp"

namespace motor {

MotorController::MotorController(IDriverPtr motorDriver, ILimiterPtr beginLimiter, ILimiterPtr endLimiter) : m_limiters(),
                                                                                                             m_motorDriver(std::move(motorDriver)),
                                                                                                             m_homeReachedCallback(nullptr),
                                                                                                             m_limitTriggeredCallback(nullptr),
                                                                                                             m_callbackFunctionForGoToLimit(nullptr),
                                                                                                             m_targetPositionReachedCallback(nullptr),
                                                                                                             m_directionOfMotion(0),
                                                                                                             m_disallowedDirection(0),
                                                                                                             m_isOnWayToHome(false),
                                                                                                             m_limitSwitchCheckPerformed(false),
                                                                                                             m_firstProcessingAfterTargetReached(true),
                                                                                                             m_targetPosition_InSteps(0),
                                                                                                             m_currentPosition_InSteps(0),
                                                                                                             m_lastStepTime_InUS(0),
                                                                                                             m_stepsPerMillimeter(25),
                                                                                                             m_stepsPerRevolution(200),
                                                                                                             m_nextStepPeriod_InUS(0),
                                                                                                             m_currentStepPeriod_InUS(0),
                                                                                                             m_periodOfSlowestStep_InUS(0),
                                                                                                             m_desiredPeriod_InUSPerStep(0),
                                                                                                             m_desiredSpeed_InStepsPerSecond(0),
                                                                                                             m_acceleration_InStepsPerUSPerUS(0),
                                                                                                             m_deceleration_InStepsPerUSPerUS(0),
                                                                                                             m_minimumPeriodForAStoppedMotion(0),
                                                                                                             m_acceleration_InStepsPerSecondPerSecond(0),
                                                                                                             m_deceleration_InStepsPerSecondPerSecond(0) {
  if (beginLimiter) {
    m_limiters.push_back(std::move(beginLimiter));
  }

  if (endLimiter) {
    m_limiters.push_back(std::move(endLimiter));
  }

  setSpeedInStepsPerSecond(200);
  setAccelerationInStepsPerSecondPerSecond(200.0);
  setDecelerationInStepsPerSecondPerSecond(200.0);
}

void MotorController::registerHomeReachedCallback(CallbackFunction homeReachedCallbackFunction) {
  m_homeReachedCallback = std::move(homeReachedCallbackFunction);
}

void MotorController::registerLimitReachedCallback(CallbackFunction limitSwitchTriggeredCallbackFunction) {
  m_limitTriggeredCallback = std::move(limitSwitchTriggeredCallbackFunction);
}

void MotorController::registerTargetPositionReachedCallback(PositionCallbackFunction targetPositionReachedCallbackFunction) {
  m_targetPositionReachedCallback = std::move(targetPositionReachedCallbackFunction);
}

int8_t MotorController::getDirectionOfMotion() const {
  return m_directionOfMotion;
}

int32_t MotorController::getTargetPositionInSteps() const {
  return m_targetPosition_InSteps;
}

int32_t MotorController::getCurrentPositionInSteps() const {
  return m_currentPosition_InSteps;
}

int32_t MotorController::getDistanceToTargetSigned() const {
  return (m_targetPosition_InSteps - m_currentPosition_InSteps);
}

uint32_t MotorController::getDistanceToTargetUnsigned() const {
  auto const distanceToTarget = getDistanceToTargetSigned();

  if (distanceToTarget < 0) {
    return static_cast<uint32_t>(-distanceToTarget);
  }

  return static_cast<uint32_t>(distanceToTarget);
}

float MotorController::getTargetPositionInMillimeters() const {
  auto const targetPosition_InSteps = getTargetPositionInSteps();
  auto const targetPosition_InMillimeters = static_cast<float>(targetPosition_InSteps) / m_stepsPerMillimeter;

  return targetPosition_InMillimeters;
}

float MotorController::getTargetPositionInRevolutions() const {
  auto const targetPosition_InSteps = getTargetPositionInSteps();
  auto const targetPosition_InRevolutions = static_cast<float>(targetPosition_InSteps) / m_stepsPerRevolution;

  return targetPosition_InRevolutions;
}

float MotorController::getCurrentPositionInMillimeters() const {
  auto const currentPosition_InSteps = getCurrentPositionInSteps();
  auto const currentPosition_InMillimeters = static_cast<float>(currentPosition_InSteps) / m_stepsPerMillimeter;

  return currentPosition_InMillimeters;
}

float MotorController::getCurrentPositionInRevolutions() const {
  auto const currentPosition_InSteps = getCurrentPositionInSteps();
  auto const currentPosition_InRevolutions = static_cast<float>(currentPosition_InSteps) / m_stepsPerRevolution;

  return currentPosition_InRevolutions;
}

float MotorController::getCurrentVelocityInStepsPerSecond() const {
  if (m_currentStepPeriod_InUS == 0.0) {
    return m_currentStepPeriod_InUS;
  }

  if (m_directionOfMotion > 0) {
    return (1000000.0f / m_currentStepPeriod_InUS);
  }

  return (-1000000.0f / m_currentStepPeriod_InUS);
}

float MotorController::getCurrentVelocityInMillimetersPerSecond() const {
  auto const currentVelocity_InStepsPerSecond = getCurrentVelocityInStepsPerSecond();
  auto const currentVelocity_InMillimetersPerSecond = currentVelocity_InStepsPerSecond / m_stepsPerMillimeter;

  return currentVelocity_InMillimetersPerSecond;
}

float MotorController::getCurrentVelocityInRevolutionsPerSecond() const {
  auto const currentVelocity_InStepsPerSecond = getCurrentVelocityInStepsPerSecond();
  auto const currentVelocity_InRevolutionsPerSecond = currentVelocity_InStepsPerSecond / m_stepsPerRevolution;

  return currentVelocity_InRevolutionsPerSecond;
}

float MotorController::getConfiguredAccelerationInStepsPerSecondPerSecond() const {
  return m_acceleration_InStepsPerSecondPerSecond;
}

float MotorController::getConfiguredAccelerationInMillimetersPerSecondPerSecond() const {
  return m_acceleration_InStepsPerSecondPerSecond / m_stepsPerMillimeter;
}

float MotorController::getConfiguredAccelerationInRevolutionsPerSecondPerSecond() const {
  return m_acceleration_InStepsPerSecondPerSecond / m_stepsPerRevolution;
}

float MotorController::getConfiguredDecelerationInStepsPerSecondPerSecond() const {
  return m_deceleration_InStepsPerSecondPerSecond;
}

float MotorController::getConfiguredDecelerationInMillimetersPerSecondPerSecond() const {
  return m_deceleration_InStepsPerSecondPerSecond / m_stepsPerMillimeter;
}

float MotorController::getConfiguredDecelerationInRevolutionsPerSecondPerSecond() const {
  return m_deceleration_InStepsPerSecondPerSecond / m_stepsPerRevolution;
}

void MotorController::setMicrostep(uint32_t microstep) {
  m_motorDriver->setMicrostep(microstep);
}

void MotorController::setStepsPerMillimeter(float const motorStepsPerMillimeter) {
  m_stepsPerMillimeter = motorStepsPerMillimeter;
}

void MotorController::setStepsPerRevolution(float const motorStepPerRevolution) {
  m_stepsPerRevolution = motorStepPerRevolution;
}

void MotorController::setSpeedInStepsPerSecond(float const speedInStepsPerSecond) {
  auto const microstep = m_motorDriver->getMicrostep();
  auto const speedInMicrostepPerSecond = speedInStepsPerSecond * static_cast<float>(microstep);

  m_desiredSpeed_InStepsPerSecond = speedInMicrostepPerSecond;
  m_desiredPeriod_InUSPerStep = 1000000.0f / m_desiredSpeed_InStepsPerSecond;
}

void MotorController::setSpeedInMillimetersPerSecond(float const speedInMillimetersPerSecond) {
  setSpeedInStepsPerSecond(speedInMillimetersPerSecond * m_stepsPerMillimeter);
}

void MotorController::setSpeedInRevolutionsPerSecond(float const speedInRevolutionsPerSecond) {
  setSpeedInStepsPerSecond(speedInRevolutionsPerSecond * m_stepsPerRevolution);
}

void MotorController::setAccelerationInStepsPerSecondPerSecond(float const accelerationInStepsPerSecondPerSecond) {
  auto const microstep = m_motorDriver->getMicrostep();
  auto const accelerationInMicrostepPerSecondPerSecond = accelerationInStepsPerSecondPerSecond * static_cast<float>(microstep);

  m_acceleration_InStepsPerSecondPerSecond = accelerationInMicrostepPerSecondPerSecond;
  m_acceleration_InStepsPerUSPerUS = m_acceleration_InStepsPerSecondPerSecond / 1E+12f;

  m_periodOfSlowestStep_InUS = 1000000.0f / std::sqrt(2.0f * m_acceleration_InStepsPerSecondPerSecond);
  m_minimumPeriodForAStoppedMotion = m_periodOfSlowestStep_InUS / 2.8f;
}

void MotorController::setAccelerationInMillimetersPerSecondPerSecond(float const accelerationInMillimetersPerSecondPerSecond) {
  setAccelerationInStepsPerSecondPerSecond(accelerationInMillimetersPerSecondPerSecond * m_stepsPerMillimeter);
}

void MotorController::setAccelerationInRevolutionsPerSecondPerSecond(float const accelerationInRevolutionsPerSecondPerSecond) {
  setAccelerationInStepsPerSecondPerSecond(accelerationInRevolutionsPerSecondPerSecond * m_stepsPerRevolution);
}

void MotorController::setDecelerationInStepsPerSecondPerSecond(float const decelerationInStepsPerSecondPerSecond) {
  auto const microstep = m_motorDriver->getMicrostep();
  auto const decelerationInMicrostepPerSecondPerSecond = decelerationInStepsPerSecondPerSecond * static_cast<float>(microstep);

  m_deceleration_InStepsPerSecondPerSecond = decelerationInMicrostepPerSecondPerSecond;
  m_deceleration_InStepsPerUSPerUS = m_deceleration_InStepsPerSecondPerSecond / 1E+12f;
}

void MotorController::setDecelerationInMillimetersPerSecondPerSecond(float const decelerationInMillimetersPerSecondPerSecond) {
  setDecelerationInStepsPerSecondPerSecond(decelerationInMillimetersPerSecondPerSecond * m_stepsPerMillimeter);
}

void MotorController::setDecelerationInRevolutionsPerSecondPerSecond(float const decelerationInRevolutionsPerSecondPerSecond) {
  setDecelerationInStepsPerSecondPerSecond(decelerationInRevolutionsPerSecondPerSecond * m_stepsPerRevolution);
}

void MotorController::setCurrentPositionInSteps(int32_t const currentPositionInSteps) {
  m_currentPosition_InSteps = currentPositionInSteps;
}

void MotorController::setCurrentPositionInMillimeters(float const currentPositionInMillimeters) {
  auto const currentPositionInSteps = currentPositionInMillimeters * m_stepsPerMillimeter;
  auto const roundedPositionInSteps = std::round(currentPositionInSteps);

  setCurrentPositionInSteps(static_cast<int32_t>(roundedPositionInSteps));
}

void MotorController::setCurrentPositionInRevolutions(float const currentPositionInRevolutions) {
  auto const currentPositionInSteps = currentPositionInRevolutions * m_stepsPerRevolution;
  auto const roundedPositionInSteps = std::round(currentPositionInSteps);

  setCurrentPositionInSteps(static_cast<int32_t>(roundedPositionInSteps));
}

void MotorController::setCurrentPositionAsHomeAndStop() {
  m_isOnWayToHome = false;
  m_currentStepPeriod_InUS = 0.0;
  m_nextStepPeriod_InUS = 0.0;
  m_directionOfMotion = 0;
  m_currentPosition_InSteps = 0;
  m_targetPosition_InSteps = 0;
}

void MotorController::setTargetPositionToStop() {
  m_isOnWayToHome = false;

  if (m_directionOfMotion == 0) {
    return;
  }

  auto const decelerationDistance_InSteps = 5E+11f / (m_deceleration_InStepsPerSecondPerSecond * m_currentStepPeriod_InUS * m_currentStepPeriod_InUS);
  auto const roundDecelerationDistance_InSteps = std::round(decelerationDistance_InSteps);

  if (m_directionOfMotion > 0) {
    setTargetPositionInSteps(m_currentPosition_InSteps + static_cast<int32_t>(roundDecelerationDistance_InSteps));
  } else {
    setTargetPositionInSteps(m_currentPosition_InSteps - static_cast<int32_t>(roundDecelerationDistance_InSteps));
  }
}

void MotorController::setTargetPositionInSteps(int32_t const absolutePositionToMoveToInSteps) {
  auto const microstep = m_motorDriver->getMicrostep();
  auto const absolutePositionToMoveToInMicrostep = static_cast<int32_t>(absolutePositionToMoveToInSteps * microstep);

  m_isOnWayToHome = false;
  m_targetPosition_InSteps = absolutePositionToMoveToInMicrostep;
  m_firstProcessingAfterTargetReached = true;
}

void MotorController::setTargetPositionInMillimeters(float const absolutePositionToMoveToInMillimeters) {
  auto const absolutePositionToMoveToInSteps = absolutePositionToMoveToInMillimeters * m_stepsPerMillimeter;
  auto const roundedPositionInSteps = std::round(absolutePositionToMoveToInSteps);

  setTargetPositionInSteps(static_cast<int32_t>(roundedPositionInSteps));
}

void MotorController::setTargetPositionInRevolutions(float const absolutePositionToMoveToInRevolutions) {
  auto const absolutePositionToMoveToInSteps = absolutePositionToMoveToInRevolutions * m_stepsPerRevolution;
  auto const roundedPositionInSteps = std::round(absolutePositionToMoveToInSteps);

  setTargetPositionInSteps(static_cast<int32_t>(roundedPositionInSteps));
}

void MotorController::setTargetPositionRelativeInSteps(int32_t const distanceToMoveInSteps) {
  setTargetPositionInSteps(m_currentPosition_InSteps + distanceToMoveInSteps);
}

void MotorController::setTargetPositionRelativeInMillimeters(float const distanceToMoveInMillimeters) {
  auto const distanceToMoveInSteps = distanceToMoveInMillimeters * m_stepsPerMillimeter;
  auto const roundedDistanceToMoveInSteps = std::round(distanceToMoveInSteps);

  setTargetPositionRelativeInSteps(static_cast<int32_t>(roundedDistanceToMoveInSteps));
}

void MotorController::setTargetPositionRelativeInRevolutions(float const distanceToMoveInRevolutions) {
  auto const distanceToMoveInSteps = distanceToMoveInRevolutions * m_stepsPerRevolution;
  auto const roundedDistanceToMoveInSteps = std::round(distanceToMoveInSteps);

  setTargetPositionRelativeInSteps(static_cast<int32_t>(roundedDistanceToMoveInSteps));
}

bool MotorController::isDriverEnabled() const {
  return m_motorDriver->isEnabled();
}

bool MotorController::isMotionComplete() const {
  if (m_directionOfMotion == 0 and m_currentPosition_InSteps == m_targetPosition_InSteps) {
    return true;
  }

  return false;
}

bool MotorController::isHomed() const {
  if (m_directionOfMotion == 0 and m_currentPosition_InSteps == 0) {
    return true;
  }

  return false;
}

void MotorController::enableDriver() {
  m_motorDriver->enable();
}

void MotorController::disableDriver() {
  m_motorDriver->disable();
}

void MotorController::startJogging(int8_t const direction) {
  setTargetPositionInSteps(direction * 2000000000);
}

void MotorController::stopJogging() {
  setTargetPositionToStop();
}

void MotorController::goToLimit(int8_t const direction, float const speedInStepsPerSecond) {
  auto const originalDesiredSpeed_InStepsPerSecond = m_desiredSpeed_InStepsPerSecond;

  setSpeedInStepsPerSecond(speedInStepsPerSecond);

  setTargetPositionInSteps(getCurrentPositionInSteps() + (direction * 2000000000));

  while (processMovement());

  setSpeedInStepsPerSecond(originalDesiredSpeed_InStepsPerSecond);
}

void MotorController::goToLimitAndSetAsHome(int8_t const direction, float const speedInStepsPerSecond) {
  auto const originalDesiredSpeed_InStepsPerSecond = m_desiredSpeed_InStepsPerSecond;

  setSpeedInStepsPerSecond(speedInStepsPerSecond);

  setTargetPositionInSteps(getCurrentPositionInSteps() + (direction * 2000000000));

  m_isOnWayToHome = true;

  while (processMovement());

  setSpeedInStepsPerSecond(originalDesiredSpeed_InStepsPerSecond);
}

void MotorController::moveToPositionInSteps(int32_t const absolutePositionToMoveToInSteps) {
  setTargetPositionInSteps(absolutePositionToMoveToInSteps);

  while (processMovement());
}

void MotorController::moveToPositionInMillimeters(float const absolutePositionToMoveToInMillimeters) {
  setTargetPositionInMillimeters(absolutePositionToMoveToInMillimeters);

  while (processMovement());
}

void MotorController::moveToPositionInRevolutions(float const absolutePositionToMoveToInRevolutions) {
  setTargetPositionInRevolutions(absolutePositionToMoveToInRevolutions);

  while (processMovement());
}

void MotorController::moveRelativeInSteps(int32_t const distanceToMoveInSteps) {
  setTargetPositionRelativeInSteps(distanceToMoveInSteps);

  while (processMovement());
}

void MotorController::moveRelativeInMillimeters(float const distanceToMoveInMillimeters) {
  setTargetPositionRelativeInMillimeters(distanceToMoveInMillimeters);

  while (processMovement());
}

void MotorController::moveRelativeInRevolutions(float const distanceToMoveInRevolutions) {
  setTargetPositionRelativeInRevolutions(distanceToMoveInRevolutions);

  while (processMovement());
}

void MotorController::moveToHome() {
  auto const distantToHome_InSteps = -getCurrentPositionInSteps();

  setTargetPositionRelativeInSteps(distantToHome_InSteps);

  while (processMovement());
}

bool MotorController::processMovement() {
  auto const distanceToTarget_Signed = m_targetPosition_InSteps - m_currentPosition_InSteps;
  auto const activeSomeLimiter = checkingActivityOfLimiters();

  if (activeSomeLimiter) {
    if (not m_limitSwitchCheckPerformed) {
      m_limitSwitchCheckPerformed = true;

      if (distanceToTarget_Signed > 0) {
        m_disallowedDirection = 1;
      }

      if (distanceToTarget_Signed < 0) {
        m_disallowedDirection = -1;
      }

      if (m_isOnWayToHome) {
        setCurrentPositionAsHomeAndStop();

        if (m_homeReachedCallback) {
          m_homeReachedCallback();
        }

        return true;
      }
    }

    if ((m_disallowedDirection == 1 and distanceToTarget_Signed > 0) or (m_disallowedDirection == -1 and distanceToTarget_Signed < 0)) {
      m_currentStepPeriod_InUS = 0.0;
      m_nextStepPeriod_InUS = 0.0;
      m_directionOfMotion = 0;
      m_targetPosition_InSteps = m_currentPosition_InSteps;

      if (m_limitTriggeredCallback) {
        m_limitTriggeredCallback();
      }

      return true;
    }
  }

  if (not activeSomeLimiter) {
    m_limitSwitchCheckPerformed = false;
  }

  if (m_directionOfMotion == 0) {
    if (distanceToTarget_Signed > 0) {
      m_directionOfMotion = 1;
      m_motorDriver->setDirection(driver::MOTOR_ROTATE_CW);
      m_nextStepPeriod_InUS = m_periodOfSlowestStep_InUS;
      m_lastStepTime_InUS = esp_timer_get_time();

      return true;
    }

    if (distanceToTarget_Signed < 0) {
      m_directionOfMotion = -1;
      m_motorDriver->setDirection(driver::MOTOR_ROTATE_CCW);
      m_nextStepPeriod_InUS = m_periodOfSlowestStep_InUS;
      m_lastStepTime_InUS = esp_timer_get_time();

      return true;
    }

    return false;
  }

  auto const currentTime_InUS = esp_timer_get_time();
  auto const periodSinceLastStep_InUS = currentTime_InUS - m_lastStepTime_InUS;
  if (periodSinceLastStep_InUS < static_cast<uint32_t>(m_nextStepPeriod_InUS)) {
    return true;
  }

  m_motorDriver->stepUp();

  m_currentPosition_InSteps += m_directionOfMotion;
  m_currentStepPeriod_InUS = m_nextStepPeriod_InUS;
  m_lastStepTime_InUS = currentTime_InUS;

  determinePeriodOfNextStep();

  m_motorDriver->stepDown();

  if (m_currentPosition_InSteps != m_targetPosition_InSteps) {
    return true;
  }

  if (m_nextStepPeriod_InUS < m_minimumPeriodForAStoppedMotion) {
    return true;
  }

  m_currentStepPeriod_InUS = 0.0;
  m_nextStepPeriod_InUS = 0.0;
  m_directionOfMotion = 0;

  if (m_firstProcessingAfterTargetReached) {
    m_firstProcessingAfterTargetReached = false;

    if (m_targetPositionReachedCallback) {
      m_targetPositionReachedCallback(m_currentPosition_InSteps);
    }
  }

  return false;
}

bool MotorController::checkingActivityOfLimiters() {
  if (m_limiters.empty()) {
    return false;
  }

  auto const allLimitersAreInactive = std::all_of(m_limiters.cbegin(), m_limiters.cend(), [](auto const &limiter) {
    return not limiter->isActive();
  });

  if (allLimitersAreInactive) {
    return false;
  }

  return true;
}

void MotorController::determinePeriodOfNextStep() {
  auto const speedUp = [this](float const currentStepPeriodSquared) {
    m_nextStepPeriod_InUS = m_currentStepPeriod_InUS - m_acceleration_InStepsPerUSPerUS * currentStepPeriodSquared * m_currentStepPeriod_InUS;

    if (m_nextStepPeriod_InUS < m_desiredPeriod_InUSPerStep) {
      m_nextStepPeriod_InUS = m_desiredPeriod_InUSPerStep;
    }
  };

  auto const slowDown = [this](float const currentStepPeriodSquared) {
    m_nextStepPeriod_InUS = m_currentStepPeriod_InUS + m_deceleration_InStepsPerUSPerUS * currentStepPeriodSquared * m_currentStepPeriod_InUS;

    if (m_nextStepPeriod_InUS > m_periodOfSlowestStep_InUS) {
      m_nextStepPeriod_InUS = m_periodOfSlowestStep_InUS;
    }
  };

  uint32_t distanceToTarget_Unsigned;
  bool targetInPositiveDirectionFlag = false;
  bool targetInNegativeDirectionFlag = false;

  auto const distanceToTarget_Signed = m_targetPosition_InSteps - m_currentPosition_InSteps;
  if (distanceToTarget_Signed >= 0L) {
    distanceToTarget_Unsigned = distanceToTarget_Signed;
    targetInPositiveDirectionFlag = true;
  } else {
    distanceToTarget_Unsigned = -distanceToTarget_Signed;
    targetInNegativeDirectionFlag = true;
  }

  auto const currentStepPeriodSquared = m_currentStepPeriod_InUS * m_currentStepPeriod_InUS;
  auto const decelerationDistance_InSteps = static_cast<int32_t>(std::round(5E+11f / (m_deceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared)));

  if (m_directionOfMotion == 1) {
    if (targetInPositiveDirectionFlag) {
      if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) or (m_nextStepPeriod_InUS < m_desiredPeriod_InUSPerStep)) {
        return slowDown(currentStepPeriodSquared);
      }

      return speedUp(currentStepPeriodSquared);
    }

    if (m_currentStepPeriod_InUS < m_periodOfSlowestStep_InUS) {
      return slowDown(currentStepPeriodSquared);
    }

    m_directionOfMotion = -1;
    m_motorDriver->setDirection(driver::MOTOR_ROTATE_CCW);

    return;
  }

  if (m_directionOfMotion == -1) {
    if (targetInNegativeDirectionFlag) {
      if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) or (m_nextStepPeriod_InUS < m_desiredPeriod_InUSPerStep)) {
        return slowDown(currentStepPeriodSquared);
      }

      return speedUp(currentStepPeriodSquared);
    }

    if (m_currentStepPeriod_InUS < m_periodOfSlowestStep_InUS) {
      return slowDown(currentStepPeriodSquared);

    }

    m_directionOfMotion = 1;
    m_motorDriver->setDirection(driver::MOTOR_ROTATE_CW);

    return;
  }
}

}
