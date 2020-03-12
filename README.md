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
L1 | Auto aim turret and run flywheel
R1 | Load ball into turret (manual and auto)
Touchpad | Toggle manual override
Triangle | Manually run turret flywheel (manual override)
Share | Disable camera overlay (manual override)

A second PS4 Controller can also be hooked up to act as a temporary drive controller in place of the Xbox Controller. To enable, set `tempController = true` in `teleopInit()` of [Robot.java](https://github.com/RandomBananazz/2020frcsuperbot/blob/betterstuff/src/main/java/frc/robot/Robot.java#L162). The controls remain equivalent to those on the Xbox Controller.

Questions: team7690amhs@gmail.com
