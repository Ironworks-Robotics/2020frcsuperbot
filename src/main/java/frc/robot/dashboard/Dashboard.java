package frc.robot.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard{
    public static void init(){

    }

    public static void periodic(){
        SmartDashboard.putBoolean(key, value);
    }
}