# ESP-FlexyStepper

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/478b7cf581664442bc75b6a87b645553)](https://app.codacy.com/manual/pkerspe/ESP-FlexyStepper?utm_source=github.com&utm_medium=referral&utm_content=pkerspe/ESP-FlexyStepper&utm_campaign=Badge_Grade_Dashboard)

This library is used to control one or more stepper motors with a ESP 32 module. The motors are accelerated and decelerated as they travel to their destination. The library has been optimized for flexible control where speeds and positions can be changed while in-motion. Based on S.Reifels FlexyStepper library.

## Features

The library provides the following features:
  - generating pulses for a connected stepper driver with a dir and step input
  - connection of emergency switch to stop all motion immendiately
  - connection of limit switches / homing switches
  - blocking and non blocking function calls possible
  - callback functions to handle events like position reached, homing complete etc.
  - can run in different modes:
    - as a service / task in the background (so you can do whatever you want in the main loop of your sketch without interfering with the stepper motion)
    - manually call the processMovement() function in the main loop (then you have to make sure your main loop completes quick enough to ensure smooth movement
    - use the blocking movement functions, that take care of calling processMovement but block the main loop for the duration of the movement

## Example

The following is an example of how to use the library as a service running in the "background" as a separate Task on the ESP32:

```#include <ESP_FlexyStepper.h>

// IO pin assignments
const int MOTOR_STEP_PIN = 33;
const int MOTOR_DIRECTION_PIN = 25;
const int EMERGENCY_STOP_PIN = 13; //define the IO pin the emergency stop switch is connected to
const int LIMIT_SWITCH_PIN = 32;   //define the IO pin where the limit switches are connected to (switches in series in normally closed setup against ground)

// Speed settings
const int DISTANCE_TO_TRAVEL_IN_STEPS = 2000;
const int SPEED_IN_STEPS_PER_SECOND = 300;
const int ACCELERATION_IN_STEPS_PER_SECOND = 800;
const int DECELERATION_IN_STEPS_PER_SECOND = 800;

// create the stepper motor object
ESP_FlexyStepper stepper;

int previousDirection = 1;

void setup()
{
  Serial.begin(115200);
  // connect and configure the stepper motor to its IO pins
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  // set the speed and acceleration rates for the stepper motor
  stepper.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
  stepper.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
  
  // Not start the stepper instance as a service in the "background" as a separate task
  // and the OS of the ESP will take care of invoking the processMovement() task regularily so you can do whatever you want in the loop function
  stepper.startAsService();
}

void loop()
{
  // just move the stepper back and forth in an endless loop
  if (stepper.getDistanceToTargetSigned() == 0)
  {
    delay(5000);
    previousDirection *= -1;
    long relativeTargetPosition = DISTANCE_TO_TRAVEL_IN_STEPS * previousDirection;
    Serial.printf("Moving stepper by %ld steps\n", relativeTargetPosition);
    stepper.setTargetPositionRelativeInSteps(relativeTargetPosition);
  }
  
  // Notice that you can now do whatever you want in the loop function without the need to call processMovement().
  // also you do not have to care if your loop processing times are too long. 
}```

## Function overview

| Function | Desciption |
| --- | --- |
| ESP_FlexyStepper() | constructor for the class to create a new instance of the ESP-FlexyStepper |
| void startAsService(void); | start ESP-FlexyStepper as a seprate task (service) in the background so it handles the calls to processMovement() for you in thebackground and you are free to do whatever you want in the main loop. *Should NOT be used in combination with the synchronous (blocking) function calls for movement* |
| void stopService(void); | stop the ESP-FlexyStepper service. Only needed if startAsService() has been called before |
| void connectToPins(byte stepPinNumber, byte directionPinNumber); | setup the pin numbers where the external stepper driver is connected to. Provide the IO Pins fro step (or pulse) pin and direction pin |
| bool isStartedAsService(void); | |
| long getTaskStackHighWaterMark(void); | |
| void setStepsPerMillimeter(float motorStepPerMillimeter); | |
| float getCurrentPositionInMillimeters(); | |
| void setCurrentPositionInMillimeters(float currentPositionInMillimeters); | |
| void setCurrentPositionInMillimeter(float currentPositionInMillimeter); | |
| void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond); | |
| void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond); | |
| void setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond); | |
| bool moveToHomeInMillimeters(signed char directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin); | |
| void moveRelativeInMillimeters(float distanceToMoveInMillimeters); | |
| void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters); | |
| void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters); | |
| void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters); | |
| float getCurrentVelocityInMillimetersPerSecond(void); | |
| long getDistanceToTargetSigned(void); | |
| void setStepsPerRevolution(float motorStepPerRevolution); | |
| void setCurrentPositionInRevolutions(float currentPositionInRevolutions); | |
| float getCurrentPositionInRevolutions(); | |
| void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond); | |
| void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond); | |
| void setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond); | |
| bool moveToHomeInRevolutions(signed char directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin); | |
| void moveRelativeInRevolutions(float distanceToMoveInRevolutions); | |
| void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions); | |
| void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions); | |
| void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions); | |
| float getCurrentVelocityInRevolutionsPerSecond(); | |
| void setCurrentPositionInSteps(long currentPositionInSteps); | |
| void setCurrentPositionAsHomeAndStop(void); | |
| long getCurrentPositionInSteps(); | |
| void setSpeedInStepsPerSecond(float speedInStepsPerSecond); | |
| void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond); | |
| void setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond); | |
| bool moveToHomeInSteps(signed char directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps, int homeSwitchPin); | |
| void moveRelativeInSteps(long distanceToMoveInSteps); | |
| void setTargetPositionRelativeInSteps(long distanceToMoveInSteps); | |
| void moveToPositionInSteps(long absolutePositionToMoveToInSteps); | |
| void setTargetPositionInSteps(long absolutePositionToMoveToInSteps); | |
| void setTargetPositionToStop(); | |
| void setDirectionToHome(signed char directionTowardHome); | configure the direction in which to move to reach the home switch |
| void setLimitSwitchActive(byte limitSwitchType); | externally trigger a limit switch acitvation |
| bool motionComplete(); | |
| float getCurrentVelocityInStepsPerSecond(); | |
| bool processMovement(void); | calculate when the next pulse needs to be send and control high/low state of the dir and pulse/step pin. *This function does not need to be called 
a.) when you started the ESP-FlexyStepper as a service using the startAsService() function, 
b.) when you called one of the blocking/synchronous movving functions* |
| int getDirectionOfMotion(void); | get the direction of the current movement. 0 if stepper not moving at the moment, 1 or -1 if the stepper is in motion |
| bool isMovingTowardsHome(void); | true if the stepper is still on the way to the home position |
| static void taskRunner(void *parameter); | this is the function that is used as the service, you do not need to call this manually ever |
