package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;

public class Drivetrain{
    /*
    private static CANSparkMax fr;
    private static CANSparkMax br;
    private static CANSparkMax fl;
    private static CANSparkMax bl;
    */

    private static WPI_VictorSPX fr;
    private static WPI_VictorSPX br;
    private static WPI_VictorSPX fl;
    private static WPI_VictorSPX bl;
    private static SpeedControllerGroup leftGroup;
    private static SpeedControllerGroup rightGroup;
    public static DifferentialDrive drivetrain;
    private static final double safetyDiv = 3;

    public static void init(boolean leftInvert, boolean rightInvert){
        // init motor controllers
        fr = new WPI_VictorSPX(Constants.CAN.driveRightMaster);
        br = new WPI_VictorSPX(Constants.CAN.driveRightSlave);
        fl = new WPI_VictorSPX(Constants.CAN.driveLeftMaster);
        bl = new WPI_VictorSPX(Constants.CAN.driveLeftSlave);

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