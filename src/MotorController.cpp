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

//
// direction signal level for "step and direction"
//
#define POSITIVE_DIRECTION LOW
#define NEGATIVE_DIRECTION HIGH

// ---------------------------------------------------------------------------------
//                                  Setup functions
// ---------------------------------------------------------------------------------

//
// constructor for the stepper class
//
MotorController::MotorController(IDriverPtr motorDriver) : m_motorDriver(std::move(motorDriver)),
                                                           m_homeLimitSwitch(nullptr),
                                                           m_homeReachedCallback(nullptr),
                                                           m_limitTriggeredCallback(nullptr),
                                                           m_callbackFunctionForGoToLimit(nullptr),
                                                           m_targetPositionReachedCallback(nullptr) {
  m_lastStepTime_InUS = 0L;
  m_stepsPerRevolution = 200L;
  m_stepsPerMillimeter = 25.0;
  m_directionOfMotion = 0;
  m_currentPosition_InSteps = 0L;
  m_targetPosition_InSteps = 0L;
  setSpeedInStepsPerSecond(200);
  setAccelerationInStepsPerSecondPerSecond(200.0);
  setDecelerationInStepsPerSecondPerSecond(200.0);
  m_currentStepPeriod_InUS = 0.0;
  m_nextStepPeriod_InUS = 0.0;
  m_isCurrentlyHomed = false;
  m_directionTowardsHome = -1;
  m_disallowedDirection = 0;
  m_activeLimitSwitch = 0; // see LIMIT_SWITCH_BEGIN and LIMIT_SWITCH_END
  m_lastStepDirectionBeforeLimitSwitchTrigger = 0;
  m_limitSwitchCheckPerformed = false;
}

void MotorController::setHomeLimitSwitch(ILimiterPtr externalAction) {
  m_homeLimitSwitch = std::move(externalAction);
}

/**
 * get the distance in steps to the currently set target position.
 * 0 is returned if the stepper is already at the target position.
 * The returned value is signed, depending on the direction to move to reach the target
 */
int32_t MotorController::getDistanceToTargetSigned() const {
  return (m_targetPosition_InSteps - m_currentPosition_InSteps);
}

/**
 *  configure the direction in which to move to reach the home position
 *  Accepts 1 or -1 as allowed values. Other values will be ignored
 */
void MotorController::setDirectionToHome(int8_t directionTowardHome) {
  if (directionTowardHome == 0) {
    return;
  }

  m_directionTowardsHome = directionTowardHome;
}

/**
 * Notification of an externally detected limit switch activation
 * Accepts LIMIT_SWITCH_BEGIN (-1) or LIMIT_SWITCH_END (1) as parameter values to indicate
 * whether the limit switch near the begin (direction of home position) or at the end of the movement has ben triggered.
 * It is strongly recommended to perform debouncing before calling this function to prevent issues when button is released and re-triggering the limit switch function
 */
void MotorController::setLimitSwitchActive(int8_t limitSwitchType) {
  if (limitSwitchType == LIMIT_SWITCH_BEGIN or limitSwitchType == LIMIT_SWITCH_END or limitSwitchType == LIMIT_SWITCH_COMBINED_BEGIN_AND_END) {
    m_activeLimitSwitch = limitSwitchType;
    m_limitSwitchCheckPerformed = false; // set flag for newly set limit switch trigger
    if (m_limitTriggeredCallback) {
      m_limitTriggeredCallback(); // TODO: this function is called from within a ISR in ESPStepperMotorServer thus we should try to delay calling of the callback to the background task / process Steps function
    }
  }
}

/**
 * clear the limit switch flag to allow movement in both directions again
 */
void MotorController::clearLimitSwitchActive() {
  m_activeLimitSwitch = 0;
}

/**
 * get the current direction of motion of the connected stepper motor
 * returns 1 for "forward" motion
 * returns -1 for "backward" motion
 * returns 0 if the stepper has reached its destination position and is not moving anymore
 */
int8_t MotorController::getDirectionOfMotion() const {
  return m_directionOfMotion;
}

/**
 * returns true if the stepper is currently in motion and moving in the direction of the home position.
 * Depends on the settings of setDirectionToHome() which defines where "home" is...a rather philosophical question :-)
 */
bool MotorController::isMovingTowardsHome() const {
  return m_directionOfMotion == m_directionTowardsHome;
}

/**
 * activate (engage) the driver (if any is configured, otherwise will do nothing)
 */
void MotorController::enableDriver() {
  m_motorDriver->enable();
}

/**
 * deactivate (release) the driver (if any is configured, otherwise will do nothing)
 */
void MotorController::disableDriver() {
  m_motorDriver->disable();
}

bool MotorController::isDriverEnabled() const {
  return m_motorDriver->isEnabled();
}

// ---------------------------------------------------------------------------------
//                     Public functions with units in millimeters
// ---------------------------------------------------------------------------------

void MotorController::setStepsPerMillimeter(float motorStepsPerMillimeter) {
  m_stepsPerMillimeter = motorStepsPerMillimeter;
}

float MotorController::getCurrentPositionInMillimeters() const {
  auto const currentPosition_InSteps = getCurrentPositionInSteps();
  auto const currentPosition_InMillimeters = static_cast<float>(currentPosition_InSteps) / m_stepsPerMillimeter;

  return currentPosition_InMillimeters;
}

void MotorController::setCurrentPositionInMillimeters(float currentPositionInMillimeters) {
  setCurrentPositionInSteps((long) std::round(currentPositionInMillimeters * m_stepsPerMillimeter));
}

//
// set the maximum speed, units in millimeters/second, this is the maximum speed
// reached while accelerating
//  Enter:  speedInMillimetersPerSecond = speed to accelerate up to, units in
//            millimeters/second
//
void MotorController::setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond) {
  setSpeedInStepsPerSecond(speedInMillimetersPerSecond * m_stepsPerMillimeter);
}

//
// set the rate of acceleration, units in millimeters/second/second
//  Enter:  accelerationInMillimetersPerSecondPerSecond = rate of acceleration,
//          units in millimeters/second/second
//
void MotorController::setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond) {
  setAccelerationInStepsPerSecondPerSecond(accelerationInMillimetersPerSecondPerSecond * m_stepsPerMillimeter);
}

//
// set the rate of deceleration, units in millimeters/second/second
//  Enter:  decelerationInMillimetersPerSecondPerSecond = rate of deceleration,
//          units in millimeters/second/second
//
void MotorController::setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond) {
  setDecelerationInStepsPerSecondPerSecond(decelerationInMillimetersPerSecondPerSecond * m_stepsPerMillimeter);
}

//
// home the motor by moving until the homing sensor is activated, then set the
// position to zero, with units in millimeters
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move
//            in a negative directions
//          speedInMillimetersPerSecond = speed to accelerate up to while moving
//            toward home, units in millimeters/second
//          maxDistanceToMoveInMillimeters = unsigned maximum distance to move
//            toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool MotorController::moveToHomeInMillimeters(int8_t directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters) {
  return (moveToHomeInSteps(directionTowardHome, speedInMillimetersPerSecond * m_stepsPerMillimeter, maxDistanceToMoveInMillimeters * m_stepsPerMillimeter));
}

//
// move relative to the current position, units are in millimeters, this function
// does not return until the move is complete
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the
//          current position in millimeters
//
void MotorController::moveRelativeInMillimeters(float distanceToMoveInMillimeters) {
  setTargetPositionRelativeInMillimeters(distanceToMoveInMillimeters);

  while (not processMovement());
}

//
// setup a move relative to the current position, units are in millimeters, no
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the
//          current position in millimeters
//
void MotorController::setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters) {
  setTargetPositionRelativeInSteps((long) std::round(distanceToMoveInMillimeters * m_stepsPerMillimeter));
}

//
// move to the given absolute position, units are in millimeters, this function
// does not return until the move is complete
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to
//          move to in units of millimeters
//
void MotorController::moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters) {
  setTargetPositionInMillimeters(absolutePositionToMoveToInMillimeters);

  while (not processMovement());
}

//
// setup a move, units are in millimeters, no motion occurs until processMove()
// is called
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to
//          move to in units of millimeters
//
void MotorController::setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters) {
  setTargetPositionInSteps((long) std::round(absolutePositionToMoveToInMillimeters * m_stepsPerMillimeter));
}

float MotorController::getTargetPositionInMillimeters() const {
  return getTargetPositionInSteps() / m_stepsPerMillimeter;
}

//
// Get the current velocity of the motor in millimeters/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float MotorController::getCurrentVelocityInMillimetersPerSecond() const {
  return (getCurrentVelocityInStepsPerSecond() / m_stepsPerMillimeter);
}

/*
access the acceleration/deceleration parameters set by user
*/

float MotorController::getConfiguredAccelerationInStepsPerSecondPerSecond() const {
  return m_acceleration_InStepsPerSecondPerSecond;
}

float MotorController::getConfiguredAccelerationInRevolutionsPerSecondPerSecond() const {
  return m_acceleration_InStepsPerSecondPerSecond / m_stepsPerRevolution;
}

float MotorController::getConfiguredAccelerationInMillimetersPerSecondPerSecond() const {
  return m_acceleration_InStepsPerSecondPerSecond / m_stepsPerMillimeter;
}

float MotorController::getConfiguredDecelerationInStepsPerSecondPerSecond() const {
  return m_deceleration_InStepsPerSecondPerSecond;
}

float MotorController::getConfiguredDecelerationInRevolutionsPerSecondPerSecond() const {
  return m_deceleration_InStepsPerSecondPerSecond / m_stepsPerRevolution;
}

float MotorController::getConfiguredDecelerationInMillimetersPerSecondPerSecond() const {
  return m_deceleration_InStepsPerSecondPerSecond / m_stepsPerMillimeter;
}

// ---------------------------------------------------------------------------------
//                     Public functions with units in revolutions
// ---------------------------------------------------------------------------------

//
// set the number of steps the motor has per revolution
//
void MotorController::setStepsPerRevolution(float motorStepPerRevolution) {
  m_stepsPerRevolution = motorStepPerRevolution;
}

//
// get the current position of the motor in revolutions, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in revolutions returned
//
float MotorController::getCurrentPositionInRevolutions() const {
  return ((float) getCurrentPositionInSteps() / m_stepsPerRevolution);
}

//
// set the current position of the motor in revolutions, this does not move the
// Do not confuse this function with setTargetPositionInRevolutions(), it does not directly cause a motor movement per se.
// NOTE: if you called one of the move functions before (and by that setting a target position internally) you might experience that the motor starts to move after calling setCurrentPositionInRevolutions() in the case that the value of currentPositionInRevolutions is different from the target position of the stepper.
// If this is not intended, you should call setTargetPositionInRevolutions() with the same value as the setCurrentPositionInRevolutions() function directly before or after calling setCurrentPositionInRevolutions

void MotorController::setCurrentPositionInRevolutions(float currentPositionInRevolutions) {
  setCurrentPositionInSteps((long) std::round(currentPositionInRevolutions * m_stepsPerRevolution));
}

//
// set the maximum speed, units in revolutions/second, this is the maximum speed
// reached while accelerating
//  Enter:  speedInRevolutionsPerSecond = speed to accelerate up to, units in
//            revolutions/second
//
void MotorController::setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond) {
  setSpeedInStepsPerSecond(speedInRevolutionsPerSecond * m_stepsPerRevolution);
}

//
// set the rate of acceleration, units in revolutions/second/second
//  Enter:  accelerationInRevolutionsPerSecondPerSecond = rate of acceleration,
//          units in revolutions/second/second
//
void MotorController::setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond) {
  setAccelerationInStepsPerSecondPerSecond(accelerationInRevolutionsPerSecondPerSecond * m_stepsPerRevolution);
}

//
// set the rate of deceleration, units in revolutions/second/second
//  Enter:  decelerationInRevolutionsPerSecondPerSecond = rate of deceleration,
//          units in revolutions/second/second
//
void MotorController::setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond) {
  setDecelerationInStepsPerSecondPerSecond(decelerationInRevolutionsPerSecondPerSecond * m_stepsPerRevolution);
}

//
// home the motor by moving until the homing sensor is activated, then set the
//  position to zero, with units in revolutions
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in
//            a negative directions
//          speedInRevolutionsPerSecond = speed to accelerate up to while moving
//            toward home, units in revolutions/second
//          maxDistanceToMoveInRevolutions = unsigned maximum distance to move
//            toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool MotorController::moveToHomeInRevolutions(int8_t directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions) {
  return (moveToHomeInSteps(directionTowardHome, speedInRevolutionsPerSecond * m_stepsPerRevolution, maxDistanceToMoveInRevolutions * m_stepsPerRevolution));
}

//
// move relative to the current position, units are in revolutions, this function
// does not return until the move is complete
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the
//          current position in revolutions
//
void MotorController::moveRelativeInRevolutions(float distanceToMoveInRevolutions) {
  setTargetPositionRelativeInRevolutions(distanceToMoveInRevolutions);

  while (not processMovement());
}

//
// setup a move relative to the current position, units are in revolutions, no
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the
//            currentposition in revolutions
//
void MotorController::setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions) {
  setTargetPositionRelativeInSteps((long) std::round(distanceToMoveInRevolutions * m_stepsPerRevolution));
}

//
// move to the given absolute position, units are in revolutions, this function
// does not return until the move is complete
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to
//            move to in units of revolutions
//
void MotorController::moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions) {
  setTargetPositionInRevolutions(absolutePositionToMoveToInRevolutions);

  while (not processMovement());
}

//
// setup a move, units are in revolutions, no motion occurs until processMove()
// is called
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to
//          move to in units of revolutions
//
void MotorController::setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions) {
  setTargetPositionInSteps((long) std::round(absolutePositionToMoveToInRevolutions * m_stepsPerRevolution));
}

float MotorController::getTargetPositionInRevolutions() const {
  return getTargetPositionInSteps() / m_stepsPerRevolution;
}

//
// Get the current velocity of the motor in revolutions/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float MotorController::getCurrentVelocityInRevolutionsPerSecond() const {
  return (getCurrentVelocityInStepsPerSecond() / m_stepsPerRevolution);
}

// ---------------------------------------------------------------------------------
//                        Public functions with units in steps
// ---------------------------------------------------------------------------------

//
// set the current position of the motor in steps, this does not move the motor
// currentPositionInSteps = the new position value of the motor in steps to be set internally for the current position
// Do not confuse this function with setTargetPositionInMillimeters(), it does not directly cause a motor movement per se.
// Notes:
// This function should only be called when the motor is stopped
// If you called one of the move functions before (and by that setting a target position internally) you might experience that the motor starts to move after calling setCurrentPositionInSteps() in the case that the value of currentPositionInSteps is different from the target position of the stepper.
// If this is not intended, you should call setTargetPositionInSteps() with the same value as the setCurrentPositionInSteps() function directly before or after calling setCurrentPositionInSteps
//
void MotorController::setCurrentPositionInSteps(long currentPositionInSteps) {
  m_currentPosition_InSteps = currentPositionInSteps;
}

//
// get the current position of the motor in steps, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in steps returned
//
long MotorController::getCurrentPositionInSteps() const {
  return m_currentPosition_InSteps;
}

//
// set the maximum speed, units in steps/second, this is the maximum speed reached
// while accelerating
//  Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
void MotorController::setSpeedInStepsPerSecond(float speedInStepsPerSecond) {
  m_desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
  m_desiredPeriod_InUSPerStep = 1000000.0 / m_desiredSpeed_InStepsPerSecond;
}

//
// set the rate of acceleration, units in steps/second/second
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in
//          steps/second/second
//
void MotorController::setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond) {
  m_acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
  m_acceleration_InStepsPerUSPerUS = m_acceleration_InStepsPerSecondPerSecond / 1E12;

  m_periodOfSlowestStep_InUS = 1000000.0 / std::sqrt(2.0 * m_acceleration_InStepsPerSecondPerSecond);
  m_minimumPeriodForAStoppedMotion = m_periodOfSlowestStep_InUS / 2.8;
}

//
// set the rate of deceleration, units in steps/second/second
//  Enter:  decelerationInStepsPerSecondPerSecond = rate of deceleration, units in
//          steps/second/second
//
void MotorController::setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond) {
  m_deceleration_InStepsPerSecondPerSecond = decelerationInStepsPerSecondPerSecond;
  m_deceleration_InStepsPerUSPerUS = m_deceleration_InStepsPerSecondPerSecond / 1E12;
}

/**
 * set the current position as the home position (Step count = 0)
 */
void MotorController::setCurrentPositionAsHomeAndStop() {
  m_isOnWayToHome = false;
  m_currentStepPeriod_InUS = 0.0;
  m_nextStepPeriod_InUS = 0.0;
  m_directionOfMotion = 0;
  m_currentPosition_InSteps = 0;
  m_targetPosition_InSteps = 0;
  m_isCurrentlyHomed = true;
}

/**
 * start jogging in the direction of home (use setDirectionToHome() to set the proper direction) until the limit switch is hit, then set the position as home
 * Warning: This function requires a limit switch to be configured otherwise the motor will never stop jogging.
 * This is a non blocking function, you need make sure MotorController is started as service (use startAsService() function) or need to call the processMovement function manually in your main loop.
 */
void MotorController::goToLimitAndSetAsHome(CallbackFunction const &callbackFunctionForHome, long maxDistanceToMoveInSteps) {
  if (callbackFunctionForHome) {
    m_homeReachedCallback = callbackFunctionForHome;
  }

  // the second check basically utilizes the fact the the begin and end limit switch id is 1 respectively -1 so the values are the same as the direction of the movement when the steppers moves towards of of the limits
  if (m_activeLimitSwitch == 0 or m_activeLimitSwitch != m_directionTowardsHome) {
    setTargetPositionInSteps(getCurrentPositionInSteps() + (m_directionTowardsHome * maxDistanceToMoveInSteps));
  }

  m_isOnWayToHome = true; // set as last action, since other functions might overwrite it
}

void MotorController::goToLimit(int8_t direction, CallbackFunction const &callbackFunctionForLimit) {
  if (callbackFunctionForLimit) {
    m_callbackFunctionForGoToLimit = callbackFunctionForLimit;
  }

  if (m_activeLimitSwitch == 0) {
    setTargetPositionInSteps(getCurrentPositionInSteps() + (m_directionTowardsHome * 2000000000));
  }

  m_isOnWayToLimit = true; // set as last action, since other functions might overwrite it
}

/**
 * register a callback function to be called whenever a movement to home has been completed (does not trigger when movement passes by the home position)
 */
void MotorController::registerHomeReachedCallback(CallbackFunction homeReachedCallbackFunction) {
  m_homeReachedCallback = std::move(homeReachedCallbackFunction);
}

/**
 * register a callback function to be called whenever a
 */
void MotorController::registerLimitReachedCallback(CallbackFunction limitSwitchTriggeredCallbackFunction) {
  m_limitTriggeredCallback = std::move(limitSwitchTriggeredCallbackFunction);
}

/**
 * register a callback function to be called whenever a target position has been reached
 */
void MotorController::registerTargetPositionReachedCallback(PositionCallbackFunction targetPositionReachedCallbackFunction) {
  m_targetPositionReachedCallback = std::move(targetPositionReachedCallbackFunction);
}

/**
 * start jogging (continuous movement without a fixed target position)
 * uses the currently set speed and acceleration settings
 * to stop the motion call the stopJogging function.
 * Will also stop when the external limit switch has been triggered using setLimitSwitchActive() or when the emergencyStop function is triggered
 * Warning: This function requires either a limit switch to be configured or manual trigger of the stopJogging/setTargetPositionToStop or emergencyStop function, otherwise the motor will never stop jogging (which could of course also be an intended behavior)
 */
void MotorController::startJogging(int8_t direction) {
  setTargetPositionInSteps(direction * 2000000000);
}

/**
 * Stop jogging, basically an alias function for setTargetPositionToStop()
 */
void MotorController::stopJogging() {
  setTargetPositionToStop();
}

//
// home the motor by moving until the homing sensor is activated, then set the
// position to zero with units in steps
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in
//            a negative directions
//          speedInStepsPerSecond = speed to accelerate up to while moving toward
//            home, units in steps/second
//          maxDistanceToMoveInSteps = unsigned maximum distance to move toward
//            home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool MotorController::moveToHomeInSteps(int8_t directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps) {
  if (not m_homeLimitSwitch) {
    return false;
  }

  float originalDesiredSpeed_InStepsPerSecond;
  bool limitSwitchFlag;

  //
  // remember the current speed setting
  //
  originalDesiredSpeed_InStepsPerSecond = m_desiredSpeed_InStepsPerSecond;

  //
  // if the home switch is not already set, move toward it
  //
  if (m_homeLimitSwitch->isActive()) {
    //
    // move toward the home switch
    //
    setSpeedInStepsPerSecond(speedInStepsPerSecond);
    setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome);
    limitSwitchFlag = false;
    while (not processMovement()) {
      if (not m_homeLimitSwitch->isActive()) {
        limitSwitchFlag = true;
        m_directionOfMotion = 0;
        break;
      }
    }

    //
    // check if switch never detected
    //
    if (not limitSwitchFlag)
      return false;

    if (m_limitTriggeredCallback) {
      m_limitTriggeredCallback();
    }
  }

  std::this_thread::sleep_for(std::chrono::microseconds(25));

  //
  // the switch has been detected, now move away from the switch
  //
  setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome * -1);
  limitSwitchFlag = false;
  while (not processMovement()) {
    if (m_homeLimitSwitch->isActive()) {
      limitSwitchFlag = true;
      m_directionOfMotion = 0;
      break;
    }
  }

  std::this_thread::sleep_for(std::chrono::microseconds(25));

  //
  // check if switch never detected
  //
  if (not limitSwitchFlag) {
    return false;
  }

  //
  // have now moved off the switch, move toward it again but slower
  //
  setSpeedInStepsPerSecond(speedInStepsPerSecond / 8);
  setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome);
  limitSwitchFlag = false;
  while (not processMovement()) {
    if (not m_homeLimitSwitch->isActive()) {
      limitSwitchFlag = true;
      m_directionOfMotion = 0;
      break;
    }
  }

  std::this_thread::sleep_for(std::chrono::microseconds(25));

  //
  // check if switch never detected
  //
  if (not limitSwitchFlag) {
    return false;
  }

  //
  // successfully homed, set the current position to 0
  //
  setCurrentPositionInSteps(0L);

  setTargetPositionInSteps(0L);
  m_isCurrentlyHomed = true;
  m_disallowedDirection = directionTowardHome;
  //
  // restore original velocity
  //
  setSpeedInStepsPerSecond(originalDesiredSpeed_InStepsPerSecond);
  return true;
}

//
// move relative to the current position, units are in steps, this function does
// not return until the move is complete
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current
//          position in steps
//
void MotorController::moveRelativeInSteps(long distanceToMoveInSteps) {
  setTargetPositionRelativeInSteps(distanceToMoveInSteps);

  while (not processMovement());
}

//
// setup a move relative to the current position, units are in steps, no motion
// occurs until processMove() is called
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current
//            position in steps
//
void MotorController::setTargetPositionRelativeInSteps(long distanceToMoveInSteps) {
  setTargetPositionInSteps(m_currentPosition_InSteps + distanceToMoveInSteps);
}

//
// move to the given absolute position, units are in steps, this function does not
// return until the move is complete
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to
//            in units of steps
//
void MotorController::moveToPositionInSteps(long absolutePositionToMoveToInSteps) {
  setTargetPositionInSteps(absolutePositionToMoveToInSteps);

  while (not processMovement());
}

//
// setup a move, units are in steps, no motion occurs until processMove() is called
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to
//            in units of steps
//
void MotorController::setTargetPositionInSteps(long absolutePositionToMoveToInSteps) {
  // abort potentially running homing movement
  m_isOnWayToHome = false;
  m_isOnWayToLimit = false;
  m_targetPosition_InSteps = absolutePositionToMoveToInSteps;
  m_firstProcessingAfterTargetReached = true;
}

int32_t MotorController::getTargetPositionInSteps() const {
  return m_targetPosition_InSteps;
}

//
// setup a "Stop" to begin the process of decelerating from the current velocity
// to zero, decelerating requires calls to processMove() until the move is complete
// Note: This function can be used to stop a motion initiated in units of steps
// or revolutions
//
void MotorController::setTargetPositionToStop() {
  // abort potentially running homing movement
  m_isOnWayToHome = false;
  m_isOnWayToLimit = false;

  if (m_directionOfMotion == 0) {
    return;
  }

  long decelerationDistance_InSteps;

  //
  // move the target position so that the motor will begin deceleration now
  //
  decelerationDistance_InSteps = (long) std::round(5E11 / (m_deceleration_InStepsPerSecondPerSecond * m_currentStepPeriod_InUS * m_currentStepPeriod_InUS));

  if (m_directionOfMotion > 0)
    setTargetPositionInSteps(m_currentPosition_InSteps + decelerationDistance_InSteps);
  else
    setTargetPositionInSteps(m_currentPosition_InSteps - decelerationDistance_InSteps);
}

//
// if it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target position yet
//
bool MotorController::processMovement() {
  long distanceToTarget_Signed;

  // check if limit switch flag is active
  if (m_activeLimitSwitch != 0) {
    distanceToTarget_Signed = m_targetPosition_InSteps - m_currentPosition_InSteps;

    if (not m_limitSwitchCheckPerformed) {
      m_limitSwitchCheckPerformed = true;

      // a limit switch is active, so movement is only allowed in one direction (away from the switch)
      if (m_activeLimitSwitch == LIMIT_SWITCH_BEGIN) {
        m_disallowedDirection = m_directionTowardsHome;
      } else if (m_activeLimitSwitch == LIMIT_SWITCH_END) {
        m_disallowedDirection = m_directionTowardsHome * -1;
      } else if (m_activeLimitSwitch == LIMIT_SWITCH_COMBINED_BEGIN_AND_END) {
        // limit switches are paired together, so we need to try to figure out by checking which one it is, by using the last used step direction
        if (distanceToTarget_Signed > 0) {
          m_lastStepDirectionBeforeLimitSwitchTrigger = 1;
          m_disallowedDirection = 1;
        } else if (distanceToTarget_Signed < 0) {
          m_lastStepDirectionBeforeLimitSwitchTrigger = -1;
          m_disallowedDirection = -1;
        }
      }

      // movement has been triggered by goToLimitAndSetAsHome() function. so once the limit switch has been triggered we have reached the limit and need to set it as home
      if (m_isOnWayToHome) {
        setCurrentPositionAsHomeAndStop(); // clear isOnWayToHome flag and stop motion

        if (m_homeReachedCallback) {
          m_homeReachedCallback();
        }

        return true;
      }
    }

    // check if further movement is allowed
    if ((m_disallowedDirection == 1 and distanceToTarget_Signed > 0) or (m_disallowedDirection == -1 and distanceToTarget_Signed < 0)) {
      // limit switch is active and movement in request direction is not allowed
      m_currentStepPeriod_InUS = 0.0;
      m_nextStepPeriod_InUS = 0.0;
      m_directionOfMotion = 0;
      m_targetPosition_InSteps = m_currentPosition_InSteps;

      return true;
    }
  }

  unsigned long periodSinceLastStep_InUS;

  //
  // check if currently stopped
  //
  if (m_directionOfMotion == 0) {
    distanceToTarget_Signed = m_targetPosition_InSteps - m_currentPosition_InSteps;
    // check if target position in a positive direction
    if (distanceToTarget_Signed > 0) {
      m_directionOfMotion = 1;
      m_motorDriver->setDirection(driver::MOTOR_ROTATE_CW);
      m_nextStepPeriod_InUS = m_periodOfSlowestStep_InUS;
      m_lastStepTime_InUS = esp_timer_get_time();
      m_lastStepDirectionBeforeLimitSwitchTrigger = m_directionOfMotion;
      return false;
    }

      // check if target position in a negative direction
    else if (distanceToTarget_Signed < 0) {
      m_directionOfMotion = -1;
      m_motorDriver->setDirection(driver::MOTOR_ROTATE_CCW);
      m_nextStepPeriod_InUS = m_periodOfSlowestStep_InUS;
      m_lastStepTime_InUS = esp_timer_get_time();
      m_lastStepDirectionBeforeLimitSwitchTrigger = m_directionOfMotion;
      return false;
    } else {
      m_lastStepDirectionBeforeLimitSwitchTrigger = 0;

      return true;
    }
  }

  // determine how much time has elapsed since the last step (Note 1: this method
  // works even if the time has wrapped. Note 2: all variables must be unsigned)
  auto const currentTime_InUS = esp_timer_get_time();
  periodSinceLastStep_InUS = currentTime_InUS - m_lastStepTime_InUS;
  // if it is not time for the next step, return
  if (periodSinceLastStep_InUS < (unsigned long) m_nextStepPeriod_InUS) {
    return false;
  }

  // execute the step on the rising edge
  m_motorDriver->stepUp();

  // update the current position and speed
  m_currentPosition_InSteps += m_directionOfMotion;
  m_currentStepPeriod_InUS = m_nextStepPeriod_InUS;

  // remember the time that this step occured
  m_lastStepTime_InUS = currentTime_InUS;

  // figure out how long before the next step
  determinePeriodOfNextStep();

  // return the step line low
  m_motorDriver->stepDown();

  // check if the move has reached its final target position, return true if all
  // done
  if (m_currentPosition_InSteps == m_targetPosition_InSteps) {
    // at final position, make sure the motor is not going too fast
    if (m_nextStepPeriod_InUS >= m_minimumPeriodForAStoppedMotion) {
      m_currentStepPeriod_InUS = 0.0;
      m_nextStepPeriod_InUS = 0.0;
      m_directionOfMotion = 0;
      m_lastStepDirectionBeforeLimitSwitchTrigger = 0;

      if (m_firstProcessingAfterTargetReached) {
        m_firstProcessingAfterTargetReached = false;
        if (m_targetPositionReachedCallback) {
          m_targetPositionReachedCallback(m_currentPosition_InSteps);
        }

      }
      return true;
    }
  }
  return false;
}

// Get the current velocity of the motor in steps/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float MotorController::getCurrentVelocityInStepsPerSecond() const {
  if (m_currentStepPeriod_InUS == 0.0) {
    return (0);
  } else {
    if (m_directionOfMotion > 0) {
      return (1000000.0 / m_currentStepPeriod_InUS);
    } else {
      return (-1000000.0 / m_currentStepPeriod_InUS);
    }
  }
}

//
// check if the motor has competed its move to the target position
//  Exit:  true returned if the stepper is at the target position
//
bool MotorController::isMotionComplete() const {
  if (m_directionOfMotion == 0 and m_currentPosition_InSteps == m_targetPosition_InSteps) {
    return true;
  }

  return false;
}

//
// determine the period for the next step, either speed up a little, slow down a
// little or go the same speed
//
void MotorController::determinePeriodOfNextStep() {
  long distanceToTarget_Signed;
  long distanceToTarget_Unsigned;
  long decelerationDistance_InSteps;
  float currentStepPeriodSquared;
  bool speedUpFlag = false;
  bool slowDownFlag = false;
  bool targetInPositiveDirectionFlag = false;
  bool targetInNegativeDirectionFlag = false;

  //
  // determine the distance to the target position
  //
  distanceToTarget_Signed = m_targetPosition_InSteps - m_currentPosition_InSteps;
  if (distanceToTarget_Signed >= 0L) {
    distanceToTarget_Unsigned = distanceToTarget_Signed;
    targetInPositiveDirectionFlag = true;
  } else {
    distanceToTarget_Unsigned = -distanceToTarget_Signed;
    targetInNegativeDirectionFlag = true;
  }

  //
  // determine the number of steps needed to go from the current speed down to a
  // velocity of 0, Steps = Velocity^2 / (2 * Deceleration)
  //
  currentStepPeriodSquared = m_currentStepPeriod_InUS * m_currentStepPeriod_InUS;
  decelerationDistance_InSteps = (long) std::round(5E11 / (m_deceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared));

  //
  // check if: Moving in a positive direction & Moving toward the target
  //    (directionOfMotion == 1) && (distanceToTarget_Signed > 0)
  //
  if ((m_directionOfMotion == 1) && (targetInPositiveDirectionFlag)) {
    //
    // check if need to start slowing down as we reach the target, or if we
    // need to slow down because we are going too fast
    //
    if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) or (m_nextStepPeriod_InUS < m_desiredPeriod_InUSPerStep))
      slowDownFlag = true;
    else
      speedUpFlag = true;
  }

    //
    // check if: Moving in a positive direction & Moving away from the target
    //    (directionOfMotion == 1) && (distanceToTarget_Signed < 0)
    //
  else if ((m_directionOfMotion == 1) && (targetInNegativeDirectionFlag)) {
    //
    // need to slow down, then reverse direction
    //
    if (m_currentStepPeriod_InUS < m_periodOfSlowestStep_InUS) {
      slowDownFlag = true;
    } else {
      m_directionOfMotion = -1;
      m_motorDriver->setDirection(driver::MOTOR_ROTATE_CCW);
    }
  }

    //
    // check if: Moving in a negative direction & Moving toward the target
    //    (directionOfMotion == -1) && (distanceToTarget_Signed < 0)
    //
  else if ((m_directionOfMotion == -1) && (targetInNegativeDirectionFlag)) {
    //
    // check if need to start slowing down as we reach the target, or if we
    // need to slow down because we are going too fast
    //
    if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) || (m_nextStepPeriod_InUS < m_desiredPeriod_InUSPerStep))
      slowDownFlag = true;
    else
      speedUpFlag = true;
  }

    //
    // check if: Moving in a negative direction & Moving away from the target
    //    (directionOfMotion == -1) && (distanceToTarget_Signed > 0)
    //
  else if ((m_directionOfMotion == -1) && (targetInPositiveDirectionFlag)) {
    //
    // need to slow down, then reverse direction
    //
    if (m_currentStepPeriod_InUS < m_periodOfSlowestStep_InUS) {
      slowDownFlag = true;
    } else {
      m_directionOfMotion = 1;
      m_motorDriver->setDirection(driver::MOTOR_ROTATE_CW);
    }
  }

  //
  // check if accelerating
  //
  if (speedUpFlag) {
    //
    // StepPeriod = StepPeriod(1 - a * StepPeriod^2)
    //
    m_nextStepPeriod_InUS = m_currentStepPeriod_InUS - m_acceleration_InStepsPerUSPerUS * currentStepPeriodSquared * m_currentStepPeriod_InUS;

    if (m_nextStepPeriod_InUS < m_desiredPeriod_InUSPerStep)
      m_nextStepPeriod_InUS = m_desiredPeriod_InUSPerStep;
  }

  //
  // check if decelerating
  //
  if (slowDownFlag) {
    //
    // StepPeriod = StepPeriod(1 + a * StepPeriod^2)
    //
    m_nextStepPeriod_InUS = m_currentStepPeriod_InUS + m_deceleration_InStepsPerUSPerUS * currentStepPeriodSquared * m_currentStepPeriod_InUS;

    if (m_nextStepPeriod_InUS > m_periodOfSlowestStep_InUS)
      m_nextStepPeriod_InUS = m_periodOfSlowestStep_InUS;
  }
}

}
