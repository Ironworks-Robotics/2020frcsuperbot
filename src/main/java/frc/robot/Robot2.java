package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.dashboard.Dashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Vision;

public class Robot2 extends TimedRobot {
    /* CONTROLLERS */
    XboxController _gamepadDrive; // Xbox controller to drive, elevator
    Joystick _gamepadShoot; // PS4 controller for turret, intake
    boolean manualOverride;

    /* AUTO */
    Timer timer;

    /* GYRO */
    ADXRS450_Gyro gyroBoy = new ADXRS450_Gyro();
    double angle, rate;
    boolean gyroConnected;
    int angleNum;

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

        Vision.init();
        Motors.init();
        Drivetrain.init(false, true);
        Dashboard.init();
    }

    @Override
    public void robotPeriodic(){
        angle = gyroBoy.getAngle();
        rate = gyroBoy.getRate();
        gyroConnected = gyroBoy.isConnected();
        if (angle > 360) {
            angleNum = (int) angle % 360;
            angle -= 360 * angleNum;
          }
      
          if (angle < 0) {
            angleNum = (int) angle % -360;
            angle += 360 * (angleNum + 1);
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
        if (timer.get() <= 5) {
            Drivetrain.drivePeriodic(0.5, 0);
        } else {
            Drivetrain.drivePeriodic(0, 0);
        }

        if (timer.get() > 5) {
            if (Motors.aimTurret()){
                Motors.loadTurret(true); // begin loading turret
            } else if (timer.get() > 10){
                Motors.loadTurret(false); // stop all motors 
                Motors.flyOff();
            }
        }

        if (timer.get() > 10 && timer.get() < 14) {
            Drivetrain.drivePeriodic(-0.5, 0);
        } else {
            Drivetrain.drivePeriodic(0, 0);
        }
    }

    @Override
    public void teleopInit() {
        forward = 0;
        turn = 0;

        manualOverride = false;
    }

    @Override
    public void teleopPeriodic() {
        
        
    }



}