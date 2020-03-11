# Iron Works Robotics - Team 7690 - FRC 2020
This repository contains the code for our team's 2020 FRC season.
The entire folder itself is from a FRC Visual Studio project, and the robot code can be found in [/src/main/java/frc/robot](https://github.com/RandomBananazz/2020frcsuperbot/tree/betterstuff/src/main/java/frc/robot).



## Control Scheme
The Xbox One Controller will be used as the main driver controller. This controls the drivetrain and climb mechanism.
Control | Purpose
------- | -------
RT | Drivetrain forward
LT | Drivetrain backward
LS X-axis | Drivetrain turn
X | Toggle elevator enable
LB | Raise climb mechanism
RB | Lower climb mechanism
Menu | Toggle safety enable
View | Toggle reverse controls

The PS4 Controller will be used as the turret control.
Control | Purpose
------- | -------
RT | Intake in
LT | Intake out
RS X-axis | Turret aim
L1 | Auto aim turret
R1 | Fire turret
Touchpad | Toggle manual override
Triangle | Manually load and fire turret

A second PS4 Controller can also be hooked up to act as a temporary drive controller in place of the Xbox Controller. To enable, set `tempController = true` in `teleopInit()` of Robot.java. The controls remain equivalent to the Xbox Controller.

Questions: team7690amhs@gmail.com
