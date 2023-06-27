# ESP-FlexyStepper

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/478b7cf581664442bc75b6a87b645553)](https://app.codacy.com/manual/pkerspe/ESP-FlexyStepper?utm_source=github.com&utm_medium=referral&utm_content=pkerspe/ESP-FlexyStepper&utm_campaign=Badge_Grade_Dashboard)

This library is used to control one or more stepper motors with a ESP 32 module. The motors are accelerated and decelerated as they travel to their destination. The library has been optimized for flexible control where speeds and positions can be changed while in-motion. Based on S.Reifels FlexyStepper library.

## Features

The library provides the following features:
  - generating pulses for a connected stepper driver with a direction and step input
  - connection of emergency switch to stop all motion immediately
  - connection of limit switches / homing switches
  - blocking and non blocking function calls possible
  - callback functions to handle events like position reached, homing complete etc.
  - can run in different modes:
    - as a service / task in the background (so you can do whatever you want in the main loop of your sketch without interfering with the stepper motion)
    - manually call the processMovement() function in the main loop (then you have to make sure your main loop completes quick enough to ensure smooth movement
    - use the blocking movement functions, that take care of calling processMovement but block the main loop for the duration of the movement

## A view words on jitter in the generated step signals

Depending on your custom code and user case where you are using the ESP FlexyStepper library as a part, you might experience some jitter (unstable/uneven step signal frequency, short breaks in the movement or in general a not always smoothly running stepper motor).
The reason for this is to be found in the basic principle how the library is implemented currently and the fact that the Arduino sketch is running as a task within the FreeRTOS operating system of the ESP32 and along with some other hose keeping tasks in "parallel".
When you start the library as a service using the "startAsService(int coreNumber)" function, it will start a separate task on one of the two cores of the ESP32. It most cases it will not be the only task running on this core (no matter which one you chose) so it will need to share the available CPU time with at least one other task running on the same core.
Usually the "setup" and "loop" function of an Arduino Framework based program will run on core 1 of the ESP.
To my knowledge core 0 usually runs the Wifi/Bluetooth/BLE stack task.

What does this mean for you and how is it related to jitter you might ask?
Since multiple tasks that are running on the same core, also need to share the CPU cycles (for more on this topic of how tasks are managed in FreeRTOS, and the mysterious role of a 'Tick' this might be a good starting point: https://www.freertos.org/implementation/a00011.html) you can get into trouble and see some jitter especially with higher step signal frequencies (higher stepper speeds). The result will be a noisy, bumpy stepper movement since the ESP FlexyStepper Task is interrupted by the Operating System every now and then and thus might not be able to send the step signal in time to the IO Pin. 

What can I do now?
When starting the service using the startAsService() function you can provide a parameter to define the core number on which to pin the task to. The default value is 1.

If you have lot of code in the loop() function of you program, you can start the stepper service on Core 0 of the ESP32, this way it will not interfere with some heavy load you might be generating in the loop function BUT if you are also using the WiFi/Bluetooth/BLE stack of the ESP32, then core 0 might not be a good idea either, since the task that manages the before managed stack is usually running on core 0 of the ESP32 and with a high priority. So if you move the motor and at the same time have a lot of Wifi traffic or BT/BLE communication going on, the OS will frequently stop the execution of the ESP FlexyStepper task to yield the way for the Wireless stack. You will see more jitter the more wireless communication is going on.
If you rely on wireless communication in your project and at the same time you have some a lot of going on in the loop function (e.g. long running loops and other function calls), you might need to change your software architecture in general.
The goal is to keep the time needed to execute the code in the loop function as short as possible as long as the ESP FlexyStepper library is running as a service on Core 1. You can move in the direction of a interrupt or event based design pattern rather than having long running loops or while statements that just wait for a response or change of a pin state for example.

Decision matrix:
| Scenario | Suggestion which core to use |
| --- | --- |
| You are using Wifi / Bluetooth or BLE in your project | if you send/receive data while the motor is moving: start the service on core 1<br/>if you only send / receive data while the motor is not moving: you might get away with starting the service on core 0, if you experience jitter, start it on core 1 |
| You are NOT using Wifi / Bluetooth or BLE in your project | start the service on core 0 |
| You have a lot of code in your loop() function that takes a lot of time for each loop execution | if you are not using wireless communication: start the service on core 0<br/>if you are using wireless communication, go to line one of this decision matrix, if this does not help you will most likely need to optimize your loop() function execution time or move to a event/interrupt based design. You can also try to increase the task priority of the ESP Flexy Stepper task in ESP_FlexyStepper.cpp in the xTaskCreatePinnedToCore(...) function call |

For more details on the topic you can also have a look at the discussion in issue #4: https://github.com/pkerspe/ESP-FlexyStepper/issues/4 


## Example Code

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
  // and the OS of the ESP will take care of invoking the processMovement() task regularly so you can do whatever you want in the loop function
  stepper.startAsService(1);
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
  // also you do not have to care if your loop processing times are too long (you might experience some jitter though if you do). 
}
```

## Function overview

| Function | Desciption |
| --- | --- |
| `ESP_FlexyStepper()` | constructor for the class to create a new instance of the ESP-FlexyStepper |
| `void startAsService(int coreNumber)` | start ESP-FlexyStepper as a separate task (service) in the background on the defined CPU core defined by coreNumber parameter (valid values are 0 and 1), so it handles the calls to processMovement() for you in the background and you are free to do whatever you want (See note on jitter also if you plan to perform CPU intensive Tasks in the loop function) in the main loop. *Should NOT be used in combination with the synchronous (blocking) function calls for movement* |
| `void stopService()` | stop the ESP-FlexyStepper service. Only needed if startAsService() has been called before |
| `void connectToPins(byte stepPinNumber, byte directionPinNumber)` | setup the pin numbers where the external stepper driver is connected to. Provide the IO Pins fro step (or pulse) pin and direction pin |
| `bool isStartedAsService()` | returns true if the ESP-FlexyStepper service has been started/is running, false if not |
| `void setStepsPerMillimeter(float motorStepPerMillimeter)` | Configure the amount of steps (pulses) to the stepper driver that are needed to perform a movement of 1 mm |
| `void setStepsPerRevolution(float motorStepPerRevolution)` | Configure the amount of steps (pulses) to the stepper driver that are needed to perform a full rotation of the stepper motor |
| `void setDirectionToHome(signed char directionTowardHome)` | configure the direction in which to move to reach the home switch |
| `void setAccelerationInMillimetersPerSecondPerSecond(float accelerationInMillimetersPerSecondPerSecond)` | |
| `void setDecelerationInMillimetersPerSecondPerSecond(float decelerationInMillimetersPerSecondPerSecond)` | |
| `float getCurrentPositionInMillimeters()` | get the current, absolute position in millimeters. Requires that the library has been configured properly by using the `setStepsPerMillimeter(...)` function |
| `void setCurrentPositionInMillimeters(float currentPositionInMillimeters)` | set the register for the current position to a specific value e.g. to mark the home position. See also the NOTE in setCurrentPositionInSteps() |
| `void setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond)` | set the speed for the next movements (or the current motion if any is in progress) in mm/s. Requires prior configuration of the library using `setStepsPerMillimeter()` |
| `bool moveToHomeInMillimeters(signed char directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin)` | |
| `void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond)` | set the speed for the next movements (or the current motion if any is in progress) in revs/s. Requires prior configuration of the library using `setStepsPerRevolution()`|
| `void setSpeedInStepsPerSecond(float speedInStepsPerSecond)` | set the speed for the next movements (or the current motion if any is in progress) in revs/s. Requires prior configuration of the library using `setStepsPerMilimeter()` |
| `void setAccelerationInRevolutionsPerSecondPerSecond(float accelerationInRevolutionsPerSecondPerSecond)` | configure the acceleration in revs/second/second |
| `void setDecelerationInRevolutionsPerSecondPerSecond(float decelerationInRevolutionsPerSecondPerSecond)` | configure the deceleration in revs/second/second |
| `void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond)` | configure the acceleration in steps/second/second |
| `void setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond)` | configure the deceleration in steps/second/second |
| `void moveRelativeInMillimeters(float distanceToMoveInMillimeters)` | *Blocking call:* start relative movement. This is a blocking function, it will not return before the final position has been reached. |
| `void setTargetPositionRelativeInMillimeters(float distanceToMoveInMillimeters)` | set the target position to a relative value in mm from the current position. Requires the repeated call of processMovement() to sequentially update the stepper position or you need to start the ESP-FlexyStepper as as service using `startAsService()` and let the library to the rest for you. |
| `void moveToPositionInMillimeters(float absolutePositionToMoveToInMillimeters)` | *Blocking call:* start movement to absolute position in mm. This is a blocking function, it will not return before the final position has been reached. |
| `void setTargetPositionInMillimeters(float absolutePositionToMoveToInMillimeters)` | set the target position to an absolute value in mm. Requires the repeated call of processMovement() to sequentially update the stepper position or you need to start the ESP-FlexyStepper as as service using `startAsService()` and let the library to the rest for you. |
| `long getDistanceToTargetSigned(void)` | get the distance in steps to travel from the current position to the target position. If stepper has reached it's target position then 0 will be returned. This is a signed value, depending on the direction of the current movement |
| `void setCurrentPositionInRevolutions(float currentPositionInRevolutions)` | set the current position inr revolutions (basically assign a value to the current position). See also the NOTE in setCurrentPositionInSteps() |
| `float getCurrentPositionInRevolutions()` | get the current, absolute position in revs |
| `bool moveToHomeInRevolutions(signed char directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin)` | *Blocking call:* move to home position (max steps or until limit switch goes low. This is a blocking function, it will not return before the final position has been reached. |
| `void moveRelativeInRevolutions(float distanceToMoveInRevolutions)` | *Blocking call:* start relative movement with given number of revolutions. This is a blocking function, it will not return before the final position has been reached. |
| `void setTargetPositionRelativeInRevolutions(float distanceToMoveInRevolutions)` | set the target position to a relative value in revolutions from the current position. Requires the repeated call of processMovement() to sequentially update the stepper position or you need to start the ESP-FlexyStepper as as service using `startAsService()` and let the library to the rest for you. |
| `void moveToPositionInRevolutions(float absolutePositionToMoveToInRevolutions)` | *Blocking call:* start absolute movement in revolutions. This is a blocking function, it will not return before the final position has been reached. |
| `void setTargetPositionInRevolutions(float absolutePositionToMoveToInRevolutions)` | set the target position to an absolute value in revolutions from the current position. Requires the repeated call of processMovement() to sequentially update the stepper position or you need to start the ESP-FlexyStepper as as service using `startAsService()` and let the library to the rest for you. |
| `float getCurrentVelocityInRevolutionsPerSecond()` | return the current velocity as floating point number in Revolutions/Second *Note: make sure you configured the stepper correctly using the `setStepsPerRevolution` function before calling this function, otherwise the result might be incorrect!*|
| `float getCurrentVelocityInStepsPerSecond()` | return the current velocity as floating point number in Steps/Second |
| `float getCurrentVelocityInMillimetersPerSecond(void)` | return the current velocity as floating point number in millimeters/Second. *Note: make sure you configured the stepper correctly using the `setStepsPerMillimeter` function before calling this function, otherwise the result might be incorrect!* |
| `void setCurrentPositionInSteps(long currentPositionInSteps)` | set the register for the current position to a specific value e.g. to mark the home position. NOTE: if you called one of the move functions before (and by that setting a target position internally) you might experience that the motor starts to move after calling setCurrentPositionInSteps(currentPositionInSteps) in the case that the value of currentPositionInSteps is different from the target position of the stepper. If this is not intended, you should call setTargetPositionInSteps() with the same value as the setCurrentPositionInSteps() function directly before or after calling setCurrentPositionInSteps |
| `void setCurrentPositionAsHomeAndStop(void)` | set the current position of the stepper as the home position. This also sets the current position to 0. After performing this step you can always return to the home position by calling `setTargetPoisitionInSteps(0)`(or for blocking calls `moveToPositionInSteps(0)`) |
| `long getCurrentPositionInSteps()` | return the current position of the stepper in steps (absolute value, could also be negative if no proper homing has been performed before) |
| `bool moveToHomeInSteps(signed char directionTowardHome, float speedInStepsPerSecond, long maxDistanceToMoveInSteps, int homeSwitchPin)` | *Blocking call:* start movement in the given direction with a maximum number of steps or until the IO Pin defined by homeSwitchPin is going low (active low switch is required, since the library will configure this pin as input with internal pull-up in the current version). This is a blocking function, it will not return before the final position has been reached.|
| `void moveRelativeInSteps(long distanceToMoveInSteps)` | *Blocking call:* start movement to the given relative position from current position. This is a blocking function, it will not return before the final position has been reached. |
| `void setTargetPositionRelativeInSteps(long distanceToMoveInSteps)` | set a new, relative target position for the stepper in a non blocking way. Requires the repeated call of processMovement() to sequentially update the stepper position or you need to start the ESP-FlexyStepper as as service using `startAsService()` and let the library to the rest for you.|
| `void moveToPositionInSteps(long absolutePositionToMoveToInSteps)` | *Blocking call:* start movement to the given absolute position in steps. This is a blocking function, it will not return before the final position has been reached. |
| `void setTargetPositionInSteps(long absolutePositionToMoveToInSteps)` | set a new absolute target position in steps to move to |
| `void setTargetPositionToStop()` | start decelerating from the current position. Used to stop the current motion without performing a hard stop. Does nothing if the target position has already been reached and the stepper has come to a stop |
| `void setLimitSwitchActive(byte limitSwitchType)` | externally trigger a limit switch activation |
| `bool motionComplete()` | returns true if the target position has been reached and the motion stopped, false if the stepper is still moving |
| `bool processMovement(void)` | calculate when the next pulse needs to be send and control high/low state of the dir and pulse/step pin. *This function does not need to be called a.) when you started the ESP-FlexyStepper as a service using the startAsService() function, b.) when you called one of the blocking/synchronous moving functions* |
| `int getDirectionOfMotion(void)` | get the direction of the current movement. 0 if stepper not moving at the moment, 1 or -1 if the stepper is in motion |
| `bool isMovingTowardsHome(void)` | true if the stepper is still on the way to the home position |
| `static void taskRunner(void *parameter)` | this is the function that is used as the service, you do not need to call this manually ever |
| `getTaskStackHighWaterMark(void)` | this is used for debugging to see if the allocated stack trace of the task / service function is large enough. You can ignore this function |
