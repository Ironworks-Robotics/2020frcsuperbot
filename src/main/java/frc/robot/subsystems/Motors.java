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

    public static void init(){
        intakeBelt = new VictorSPX(Constants.CAN.intakeBelt);
        turretFly = new VictorSPX(Constants.CAN.turretFly);
        turretAim = new VictorSPX(Constants.CAN.turretAim);
        turretLoad = new VictorSPX(Constants.CAN.turretLoad);
        elevator = new VictorSPX(Constants.CAN.elevator);
    }

    public static boolean aimTurret() {
        double angle = Vision.getCurrentAngle();
        if (Vision.getTargetFound() && angle <= -Constants.angleTolerance){
            turretAim.set(ControlMode.PercentOutput, Constants.aimSpeed);
            return false;
        } else if (Vision.getTargetFound() && angle >= Constants.angleTolerance){
            turretAim.set(ControlMode.PercentOutput, -Constants.aimSpeed);
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

    public static void flyOff() {
        turretFly.set(ControlMode.PercentOutput, 0);
    }
}