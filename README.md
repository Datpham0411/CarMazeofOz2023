# CarMazeofOz2023
# About the functions provided in the library carMazeOfOz.h
# Notes: Please press the reset button on Arduino before use

1. setMotorLeft()
  - Description: The function is used to controls the left motor speed and direction of rotation.
  - Syntax: setMotorLeft(speed, direction)
  - Parameters:
    - speed: PWM value, from 0 to 255, to control the speed of the left motor.
    - direction: 1 or 0, to control the direction of rotation of the left motor, with 1 being the value for setting the motor to go forward, 0 being the value for setting the motor to go backwards.
  - Returns: Nothing.

2. setMotorRight()
  - Description: The function is used to controls the right motor speed and direction of rotation.
  - Syntax: setMotorRight(speed, direction)
  - Parameters:
    - speed: PWM value, from 0 to 255, to control the speed of the right motor.
    - direction: 1 or 0, to control the direction of rotation of the right motor with 1 being the value for setting the motor to go forward, 0 being the value for setting the motor to go backwards.
  - Returns: Nothing.

3. getSpeedLeft()
  - Description: The function performs the calculation of the left motor speed value.
  - Syntax: getSpeedLeft()
  - Parameters: Nothing.
  - Returns: Speed value of left motor, float type, in cm/s.

4. getSpeedRight()
  - Description: The function performs the calculation of the right motor speed value.
  - Syntax: getSpeedLeft()
  - Parameters: Nothing.
  - Returns: Speed value of right motor, float type, in cm/s. 

5. getDistanceHead()
  - Description: The function performs the calculation of the distance value in front of the car.
  - Syntax: getDistanceHead()
  - Parameters: Nothing.
  - Returns: Distance value in front of the vehicle, float type, in cm.

6. getDistanceLeft()
  - Description: The function performs the calculation of the distance value on the left side of the car.
  - Syntax: getDistanceLeft()
  - Parameters: Nothing.
  - Returns: Distance value on the left side the vehicle, float type, in cm.

7. getDistanceRight()
  - Description: The function performs the calculation of the distance value on the right side of the car.
  - Syntax: getDistanceRight()
  - Parameters: Nothing.
  - Returns: Distance value on the right side the vehicle, float type, in cm.
