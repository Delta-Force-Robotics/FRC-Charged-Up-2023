# **FRC-Charged-Up-2023**
Our repository for the 2023 FIRST Robotics Competition Season "Charged Up".

[FRC_TRI](https://github.com/Delta-Force-Robotics/FRC-Charged-Up-2023/assets/115287980/dee0b88f-4816-44c3-80a9-d17d97d475b8)

### **HIGHLIGHTS** 

* Field-Relative Swerve Drive
* 2 Game Piece Autonomous Modes
* Gyro-Based Autonomous Dock and Engage
* Automatic Elevator and Intake Retraction
* April Tag Detection and Follower

### **MAJOR PACKAGE FUNCTIONS**

* [RobotContainer](src/main/java/frc/robot/RobotContainer.java)

 Contains the general code for both TeleOperated and Autonomous period, meaning all button bindings, autonomous routine chooser and code for each autonomous trajectory sequence. 

* [Commands](src/main/java/frc/robot/Commands)

Contains code for all Commands, with various command sequences used for robot control or during the autonomous period, such as ParkCommand, AutoRetractCommand, SwerveJoystick Command etc.

Commands extend the Command abstract class.

* [AutoRetractCommand](src/main/java/frc/robot/Commands/AutoRetractCommand.java)

Contains code for automatic retraction of the Elevator and Intake Subsystems. It is activated while taking or scoring an element. 

* [AutoFollowingCommand](src/main/java/frc/robot/Commands/AutoFollowingCommand.java)

Contains code for April Tag Detection and Following.

* [ParkCommand](src/main/java/frc/robot/Commands/ParkCommand.java)

Contains code for robot dock and engagement during the autonomous period.

* [Subsystems](src/main/java/frc/robot/Subsystems)
  
Contains code for all subsystems with one single implementation per subsystem, such as Elevator Subsystem, Intake Subsystem, Swerve Subsystem etc.

Subsystems extend the Subsystem abstract class.

* [ElevatorSubsystem](src/main/java/frc/robot/Subsystems/ElevatorSubsystem.java)

Contains code for the Elevator subsystem with an implemented PID Controller, Trapezoid Profile, and other necessary functions.

* [LEDSubsystem](src/main/java/frc/robot/Subsystems/LEDSubsystem.java)
  
Contains code for the LED subsystem that sets the LEDs to a colour according to the current game element obtained.

* [AutoTrajectories](PathWeaver/deploy/pathplanner/generatedJSON)

Contains all autonomous paths made using PathPlanner. All paths are named according to location, starting element and number of elements scored.
