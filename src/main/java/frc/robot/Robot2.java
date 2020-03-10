package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.dashboard.Dashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Vision;

public class Robot2 extends TimedRobot {
    /* PDP */
    PowerDistributionPanel pdp;
    double[] current;

    /* CONTROLLERS */
    XboxController _gamepadDrive; // Xbox controller to drive, elevator
    Joystick _gamepadShoot; // PS4 controller for turret, intake

    /* AUTO */
    Timer timer;

    /* GYRO */
    ADXRS450_Gyro gyroBoy = new ADXRS450_Gyro();
    double angle, rate;
    boolean gyroConnected;

    /* DRIVE */
    double forward, turn;

    /* PERIPHERALS */
    DigitalInput irSensor;
    UsbCamera camera;

    @Override
    public void robotInit(){
        _gamepadDrive = new XboxController(Constants.JOYSTICKS.xbox);
        _gamepadShoot = new Joystick(Constants.JOYSTICKS.ps4);
        
        gyroBoy.calibrate();

        timer = new Timer();
        
        camera = CameraServer.getInstance().startAutomaticCapture(0);

        pdp = new PowerDistributionPanel();
        current = new double[16];

        Vision.init();
        Motors.init();
        Drivetrain.init(false, true);
        Dashboard.init();
    }

    @Override
    public void robotPeriodic(){
        for (int i = 0; i < current.length; i++) {
            current[i] = pdp.getCurrent(i);
        }

        Vision.periodic();
        Dashboard.periodic();
    }

    @Override
    public void autonomousInit(){
        timer.reset();
        timer.start();
    }

    @Override
    public void autonomousPeriodic(){
        if (timer.get() < 5) {
            Drivetrain.drivePeriodic(0.5, 0);
        } else {
            Drivetrain.drivePeriodic(0, 0);
        }

        if (timer.get() > 5) {
            
        }
    }



}