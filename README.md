# **WiredDevils5498 FRC Robot Code**

This repository contains the source code for the **WiredDevils5498** FRC (FIRST Robotics Competition) robot. The code is written in Java and is designed to run on a RoboRIO (the FRC robot controller). It includes functionality for the robot's drivetrain, arm control, and other subsystems necessary for the competition.

---

## **Table of Contents**

- [Robot Overview](#robot-overview)
- [Features](#features)
- [Controls](#controls)
- [How to Use](#how-to-use)
- [Contributing](#contributing)
- [License](#license)

---

## **Robot Overview**

The **WiredDevils5498** robot features a **swerve drive system** that allows precise control over movement. This system enables the robot to move in any direction while also rotating independently. The robot also includes various subsystems such as:
- **Arm control**: For manipulating game pieces.
- **Shooter**: To launch game pieces to scoring zones.
- **Climber**: For robot mobility and climbing tasks.

### **Subsystems**
- **Swerve Drive**: Enables the robot to move in any direction, including full rotation.
- **ArmStuff**: Controls the arm's actuators, including a shooter and a picker-upper for handling game pieces.
- **Climber**: Provides functionality to manage the robot's climbing mechanism.
- **Camera System**: Handles camera feeds used for vision processing (if applicable).

---

## **Features**

- **Autonomous Mode**: The robot can execute a set of predefined actions using **PathPlannerLib** for path following.
- **Teleoperated Mode**: Controlled via **joysticks** or **gamepad** input for manual operation.
- **Subsystem Control**: The robot can control multiple subsystems (arm, shooter, etc.) based on input from the driver.

---

## **Setting Constants**

The following things must be adjusted to your robot and module's specific constants in the Constants.java file (all distance units must be in meters, and rotation units in radians):</br>
These instructions are mostly followable from Step 
1. Gyro Settings: ```pigeonID``` and ```invertGyro``` (ensure that the gyro rotation is CCW+ (Counter Clockwise Positive)
2. ```chosenModule```: 
<br>If you are using a COTS SDS Module (more modules will be added in the future), set the module and drive ratio you are using here. 
<br>This will automatically set certain constants for the specific module required to function properly. 
<br><b><u>If you are not using a COTS supported module, you should delete this variable, and fix all the errors that pop up with correct values for the module you are using</b></u>
<br> Here is a list of the constants that will automatically be set if you are using a supported module:
    * Wheel Circumference
    * Angle Motor Invert
    * Drive Motor Invert
    * CANCoder Sensor Invert
    * Angle Motor Gear Ratio
    * Drive Motor Gear Ratio
    * Angle Falcon Motor PID Values
    
3. ```trackWidth```: Center to Center distance of left and right modules in meters.
4. ```wheelBase```: Center to Center distance of front and rear module wheels in meters.
5. ```wheelCircumference```: Cirumference of the wheel (including tread) in meters. <br><b>If you are using a supported module, this value will be automatically set.</b>
6. ```driveGearRatio```: Total gear ratio for the drive motor. <br><b>If you are using a supported module, this value will be automatically set.</b>
7. ```angleGearRatio```: Total gear ratio for the angle motor. <br><b>If you are using a supported module, this value will be automatically set.</b>
8. ```canCoderInvert``` and ```angleMotorInvert```: Both must be set such that they are CCW+. <br><b>If you are using a supported module, this value will be automatically set.</b>
9. ```driveMotorInvert```: <b>If you are using a supported module, this value will be automatically set.</b>
<br>This can always remain false, since you set your offsets in step 11 such that a positive input to the drive motor will cause the robot to drive forwards.
<br>However this can be set to true if for some reason you prefer the bevel gears on the wheel to face one direction or another when setting offsets. See Step 11 for more information.

10. ```Module Specific Constants```: set the Can Id's of the motors and CANCoders for the respective modules, see the next step for setting offsets.
11. Setting Offsets
    * For finding the offsets, use a piece of 1x1 metal that is straight against the forks of the front and back modules (on the left and right side) to ensure that the modules are straight. 
    * Point the bevel gears of all the wheels in the same direction (either facing left or right), where a postive input to the drive motor drives the robot forward (you can use phoenix tuner to test this). If for some reason you set the offsets with the wheels backwards, you can change the ```driveMotorInvert``` value to fix.
    * Open smartdashboard (or shuffleboard and go to the smartdashboard tab), you will see 4 printouts called "Mod 0 Cancoder", "Mod 1 Cancoder", etc. 
    <br>If you have already straightened the modules, copy those 4 numbers exactly (to 2 decimal places) to their respective ```angleOffset``` variable in constants.
    <br><b>Note:</b> The CANcoder values printed to smartdashboard are in degrees, when copying the values to ```angleOffset``` you must use ```Rotation2d.fromDegrees("copied value")```.

12. Angle Motor PID Values: <br><b>If you are using a supported module, this value will be automatically set. If you are not, or prefer a more or less aggressive response, you can use the below instructions to tune.</b> 
    * To tune start with a low P value (0.01).
    * Multiply by 2 until the module starts oscilating around the set point
    * Scale back by searching for the value (for example, if it starts oscillating at a P of 10, then try (10 -> 5 -> 7.5 -> etc)) until the module doesn't oscillate around the setpoint.
    * If there is any overshoot you can add in some D by repeating the same process, leave at 0 if not. Always leave I at 0.

13. ```maxSpeed```: In Meters Per Second. ```maxAngularVelocity```: In Radians Per Second. For these you can use the theoretical values, but it is better to physically drive the robot and find the actual max values.


14. Get the drive characterization values (KS, KV, KA) by using the WPILib characterization tool, found [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html). You will need to lock your modules straight forward, and complete the characterization as if it was a standard tank drive.
15. ```driveKP```: 
<br>After completeing characterization and inserting the KS, KV, and KA values into the code, tune the drive motor kP until it doesn't overshoot and doesnt oscilate around a target velocity.
<br>Leave ```driveKI```, ```driveKD```, and ```driveKF``` at 0.0.

---

## **Controls**

### **Driver Controls (Joystick 0)** (Using Xbox Controller):
- **Left Stick (Y-Axis)**: Move the robot forward/backward (Translation).
- **Left Stick (X-Axis)**: Move the robot left/right (Strafe).
- **Right Stick (X-Axis)**: Rotate the robot.
- **X Button**: Reset the gyro.
- **B Button**: Extend actuator.
- **Left Bumper**: Retract actuator.
- **Right Bumper**: Dampen robot speed.
- **A Button**: Drive to April tag (Auto Mode).
- **Left Trigger**: Control climb (down).
- **Right Trigger**: Control climb (retract).
- **POV Up (90°)**: Lock robot heading to 90°.
- **POV Down (270°)**: Lock robot heading to 270°.
- **POV Left (180°)**: Lock robot heading to 180°.
- **POV Right (0°)**: Lock robot heading to 0°.

### **Upper Controls (Joystick 1)** (Using Xbox Controller):
- **Right Stick (Y-Axis)**: Move the wrist of the robot.
- **Left Stick (Y-Axis)**: Move the elevator up/down.
- **B Button**: Elevator SetPoint1.
- **Y Button**: Set wrist to preset position (WristSetPoint1).
- **X Button**: Set wrist to another preset position (WristSetPoint2).
- **Left Bumper**: Shooter reverse (shooter backward).
- **Right Bumper**: Shooter forward (shooter forward).

---

## **How to Use**

1. **Clone the repository** to your local machine.
2. **Build the project** using Gradle (`./gradlew build`).
3. **Deploy** the code to the RoboRIO using WPILib’s tools.
4. **Control the robot** through the driver and upper controls during the competition or testing.

---

## **License**

Copyright (c) 2009-2024 FIRST and other WPILib contributors All rights reserved - see the [LICENSE.md](WPILib-License.md) file for details.
