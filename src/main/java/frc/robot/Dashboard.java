package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class Dashboard{
    public static void update(){
        SmartDashboard.putBoolean("Gyro Connected", Robot.gyroConnected);
        SmartDashboard.putNumber("Gyro Angle", Robot.angle);
        SmartDashboard.putNumber("Gyro Rate", Robot.rate);

        SmartDashboard.putBoolean("Safety", Robot.safety);
        SmartDashboard.putBoolean("Reverse", Robot.reverse);

        SmartDashboard.putBoolean("Elevator Enabled", Robot.enableElevator);
        SmartDashboard.putBoolean("Manual Override", Robot.manualOverride);

        SmartDashboard.putNumber("Vision Angle", Vision.getCurrentAngle());
        SmartDashboard.putBoolean("Vision Target Found", Vision.getTargetFound());
        SmartDashboard.putBoolean("Vision Target Locked", Robot.locked);

        SmartDashboard.putBoolean("Ball Detected", Robot.irSensor.get());
    }
}