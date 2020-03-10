package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Drivetrain{
    private static CANSparkMax fr;
    private static CANSparkMax br;
    private static CANSparkMax fl;
    private static CANSparkMax bl;
    private static SpeedControllerGroup leftGroup;
    private static SpeedControllerGroup rightGroup;
    private static DifferentialDrive drivetrain;

    public static DifferentialDrive init(boolean leftInvert, boolean rightInvert){
        fr = new CANSparkMax(Constants.CAN.driveRightMaster, MotorType.kBrushless);
        br = new CANSparkMax(Constants.CAN.driveRightSlave, MotorType.kBrushless);
        fl = new CANSparkMax(Constants.CAN.driveLeftMaster, MotorType.kBrushless);
        bl = new CANSparkMax(Constants.CAN.driveLeftSlave, MotorType.kBrushless);

        leftGroup = new SpeedControllerGroup(fl, bl);
        rightGroup = new SpeedControllerGroup(fr, br);

        leftGroup.setInverted(leftInvert);
        rightGroup.setInverted(rightInvert);

        drivetrain = new DifferentialDrive(leftGroup, rightGroup);
        
        return drivetrain;
    }
}