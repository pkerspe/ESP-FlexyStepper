
//      ******************************************************************
//      *                                                                *
//      *                    Header file for ESP-FlexyStepper            *
//      *                                                                *
//      *            Paul Kerspe                     4.6.2020            *
//      *       based on the concept of FlexyStepper by Stan Reifel      *
//      *                                                                *
//      ******************************************************************

// MIT License
//
// Copyright (c) 2020 Paul Kerspe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This library is based on the works of Stan Reifel in his FlexyStepper library:
// https://github.com/Stan-Reifel/FlexyStepper

#ifndef ESP_FlexyStepper_h
#define ESP_FlexyStepper_h

#include <Arduino.h>
#include <stdlib.h>

class ESP_FlexyStepper
{
public:
  ESP_FlexyStepper();
  void startAsService(void);
  void stopService(void);
  void connectToPins(byte stepPinNumber, byte directionPinNumber);
  static void taskRunner(void *parameter);
  bool isStartedAsService(void);
  long getTaskStackHighWaterMark(void);

  void setStepsPerMillimeter(float motorStepPerMillimeter);
  float getCurrentPositionInMillimeters();
  void setCurrentPositionInMillimeters(float currentPositionInMillimeters);
  void setCurrentPositionInMillimeter(float currentPositionInMillimeter);
  void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond);
  void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond);
  void setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond);
  bool moveToHomeInMillimeters(signed char directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin);
  void moveRelativeInMillimeters(float distanceToMoveInMillimeters);
  void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters);
  void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters);
  float getCurrentVelocityInMillimetersPerSecond(void);
  long getDistanceToTargetSigned(void);

  void setStepsPerRevolution(float motorStepPerRevolution);
  void setCurrentPositionInRevolutions(float currentPositionInRevolutions);
  float getCurrentPositionInRevolutions();
  void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond);
  void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond);
  void setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond);
  bool moveToHomeInRevolutions(signed char directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin);
  void moveRelativeInRevolutions(float distanceToMoveInRevolutions);
  void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions);
  void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions);
  float getCurrentVelocityInRevolutionsPerSecond();

  void setCurrentPositionInSteps(long currentPositionInSteps);
  void setCurrentPositionAsHomeAndStop(void);
  long getCurrentPositionInSteps();
  void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
  void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
  void setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond);
  bool moveToHomeInSteps(signed char directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps, int homeSwitchPin);
  void moveRelativeInSteps(long distanceToMoveInSteps);
  void setTargetPositionRelativeInSteps(long distanceToMoveInSteps);
  void moveToPositionInSteps(long absolutePositionToMoveToInSteps);
  void setTargetPositionInSteps(long absolutePositionToMoveToInSteps);
  void setTargetPositionToStop();
  // configure the direction in which to move to reach the home switch
  void setDirectionToHome(signed char directionTowardHome);
  // externally trigger a limit switch acitvation
  void setLimitSwitchActive(byte limitSwitchType);
  // externally clear limit switch flag, to allow movement again
  void clearLimitSwitchActive(void);
  bool motionComplete();
  float getCurrentVelocityInStepsPerSecond();
  bool processMovement(void);
  int getDirectionOfMotion(void);
  bool isMovingTowardsHome(void);

  void emergencyStop(bool holdUntilReleased = false);
  void releaseEmergencyStop(void);

  static const byte LIMIT_SWITCH_BEGIN = -1;
  static const byte LIMIT_SWITCH_END = 1;
  static const byte LIMIT_SWITCH_COMBINED_BEGIN_AND_END = 2;

private:
  void DeterminePeriodOfNextStep();

  byte stepPin;
  byte directionPin;
  float stepsPerMillimeter;
  float stepsPerRevolution;
  int directionOfMotion;
  long currentPosition_InSteps;
  long targetPosition_InSteps;
  float desiredSpeed_InStepsPerSecond;
  float desiredPeriod_InUSPerStep;
  float acceleration_InStepsPerSecondPerSecond;
  float acceleration_InStepsPerUSPerUS;
  float deceleration_InStepsPerSecondPerSecond;
  float deceleration_InStepsPerUSPerUS;
  float periodOfSlowestStep_InUS;
  float minimumPeriodForAStoppedMotion;
  float nextStepPeriod_InUS;
  unsigned long lastStepTime_InUS;
  float currentStepPeriod_InUS;
  bool emergencyStopActive;
  bool holdEmergencyStopUntilExplicitRelease;
  signed char directionTowardsHome;
  signed char lastStepDirectionBeforeLimitSwitchTrigger;
  //true if the current stepper positon equals the homing position
  bool isCurrentlyHomed;
  signed char activeLimitSwitch;
  bool limitSwitchCheckPeformed;
  // 0 if the the stepper is allowed to move in both directions (e.g. no limit or homing switch triggered), otherwise indicated which direction is currently not allowed for further movement
  signed char disallowedDirection;

  TaskHandle_t xHandle = NULL;
};

// ------------------------------------ End ---------------------------------
#endif
