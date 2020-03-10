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
    public static DifferentialDrive drivetrain;
    private static final double safetyDiv = 4;

    public static void init(boolean leftInvert, boolean rightInvert){
        // init motor controllers
        fr = new CANSparkMax(Constants.CAN.driveRightMaster, MotorType.kBrushed);
        br = new CANSparkMax(Constants.CAN.driveRightSlave, MotorType.kBrushed);
        fl = new CANSparkMax(Constants.CAN.driveLeftMaster, MotorType.kBrushed);
        bl = new CANSparkMax(Constants.CAN.driveLeftSlave, MotorType.kBrushed);

        leftGroup = new SpeedControllerGroup(fl, bl);
        rightGroup = new SpeedControllerGroup(fr, br);

        leftGroup.setInverted(leftInvert);
        rightGroup.setInverted(rightInvert);

        drivetrain = new DifferentialDrive(leftGroup, rightGroup);
    }

    public static void disable(){
        fr.set(0);
        br.set(0);
        fl.set(0);
        bl.set(0);
    }

    public static void drivePeriodic(double forward, double turn, boolean safety, boolean reverse){
        if (safety) {
            forward /= safetyDiv;
            turn /= safetyDiv;
        }
        if (reverse) {
            forward *= -1;
            turn *= -1;
        }
        drivetrain.arcadeDrive(forward, turn);
    }

    public static void drivePeriodic(double forward, double turn){
        drivetrain.arcadeDrive(forward, turn);
    }
}