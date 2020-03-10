package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Motors{
    public static VictorSPX intakeBelt;
    public static VictorSPX turretFly;
    public static VictorSPX turretAim;
    public static VictorSPX turretLoad;
    public static VictorSPX elevator;
    public static Timer motorTimer;
    public static final double angleTolerance = 5; // Deadzone for turret aim
    public static final double aimSpeed = 0.25; // Default auto aim speed

    public static void init(){
        intakeBelt = new VictorSPX(Constants.CAN.intakeBelt);
        turretFly = new VictorSPX(Constants.CAN.turretFly);
        turretAim = new VictorSPX(Constants.CAN.turretAim);
        turretLoad = new VictorSPX(Constants.CAN.turretLoad);
        elevator = new VictorSPX(Constants.CAN.elevator);
    }

    public static boolean aimTurret() {
        double angle = Vision.getCurrentAngle();
        if (Vision.getTargetFound() && angle <= -angleTolerance){
            turretAim.set(ControlMode.PercentOutput, aimSpeed);
            return false;
        } else if (Vision.getTargetFound() && angle >= angleTolerance){
            turretAim.set(ControlMode.PercentOutput, -aimSpeed);
            return false;
        } else if (Vision.getTargetFound()) {
            turretFly.set(ControlMode.PercentOutput, 1);
            return true;
        }
        return false;
    }

    public static void loadTurret(boolean on) {
        if (on) {
            intakeBelt.set(ControlMode.PercentOutput, 1);
            turretLoad.set(ControlMode.PercentOutput, 1);
        } else {
            intakeBelt.set(ControlMode.PercentOutput, 0);
            turretLoad.set(ControlMode.PercentOutput, 0);
        }
    }

    public static void aimOff() {
        turretFly.set(ControlMode.PercentOutput, 0);
        turretAim.set(ControlMode.PercentOutput, 0);
    }

    public static void liftElevator(boolean RB, boolean LB) {
        if (RB && !LB) {
            elevator.set(ControlMode.PercentOutput, -1); // RB to drop
        } else if (LB && !RB) {
            elevator.set(ControlMode.PercentOutput, 1); // LB to lift
        } else {
            elevator.set(ControlMode.PercentOutput, 0);
        }
    }

    public static void intakePeriodic(boolean sensor) {
        intakeBelt.set(ControlMode.PercentOutput, sensor ? 1 : 0); // start intake upon ball detection
    }

    public static void manualAim(double speed) {
        speed = Constants.deadband(speed);
        if (speed == 0) {
            turretAim.set(ControlMode.PercentOutput, 0);
        } else {
            turretAim.set(ControlMode.PercentOutput, (speed < 0) ? -aimSpeed : aimSpeed);
        }
    }

    public static void manualIntake(double speed) {
        intakeBelt.set(ControlMode.PercentOutput, speed);
    }

    public static void manualFly(double speed) {
        turretFly.set(ControlMode.PercentOutput, speed);
    }

    public static void manualLoad(double speed) {
        turretLoad.set(ControlMode.PercentOutput, speed);
    }
}