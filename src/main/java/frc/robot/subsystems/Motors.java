package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;

public class Motors{
    public static VictorSPX intakeBelt;
    public static VictorSPX turretFly;
    public static VictorSPX turretAim;
    public static VictorSPX turretLoad;
    public static VictorSPX elevator;

    public static void init(){
        intakeBelt = new VictorSPX(Constants.CAN.intakeBelt);
        turretFly = new VictorSPX(Constants.CAN.turretFly);
        turretAim = new VictorSPX(Constants.CAN.turretAim);
        turretLoad = new VictorSPX(Constants.CAN.turretLoad);

        elevator = new VictorSPX(Constants.CAN.elevator);
    }
}