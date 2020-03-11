package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Dashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
    /* CONTROLLERS */
    XboxController _gamepadDrive; // Xbox controller to drive, elevator
    Joystick _gamepadShoot; // PS4 controller for turret, intake
    Joystick _gamepadTemp; // PS4 controller temporary

    /* AUTO */
    Timer timer;

    /* GYRO */
    ADXRS450_Gyro gyroBoy;
    public static double angle, rate;
    public static boolean gyroConnected;
    int angleNum;

    /* DRIVE AND TURRET*/
    public static double forward, turn;
    public static boolean safety, reverse, enableElevator, manualOverride, locked;
    public static double intakeSpeed, aimSpeed;
    public static boolean tempController;

    /* PERIPHERALS */
    UsbCamera camera;

    /* CONTROLS */
    double xboxLT, xboxRT, xboxLS, xboxRS;
    public static boolean xboxLB, xboxRB, xboxStartPressed, xboxBackPressed, xboxXPressed;
    boolean ps4L1, ps4R1, ps4TouchpadPressed, ps4Triangle;
    double ps4L2, ps4R2, ps4L3, ps4R3;
    GenericHID.Hand Left = GenericHID.Hand.kLeft;
    GenericHID.Hand Right = GenericHID.Hand.kRight;

    @Override
    public void robotInit(){
        /* CONTROLLER INIT */
        _gamepadDrive = new XboxController(Constants.JOYSTICKS.xbox);
        _gamepadShoot = new Joystick(Constants.JOYSTICKS.ps4);
        _gamepadTemp = new Joystick(Constants.JOYSTICKS.temp);

        /* GYRO INIT*/
        gyroBoy = new ADXRS450_Gyro();
        gyroBoy.calibrate();


        timer = new Timer();
        camera = CameraServer.getInstance().startAutomaticCapture(0);

        /* SUBSYSTEMS */
        Vision.init();
        Motors.init();
        Drivetrain.init(false, false);
    }

    @Override
    public void robotPeriodic(){
        /* GYRO */
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
        
        /* SUBSYSTEMS */
        Vision.periodic();
        Dashboard.update();

        /* CONTROLLER INPUT DETECTION */
        // Currently, all stick axis inputs are only horizontal (because vertical is unused)
        xboxLS = _gamepadDrive.getX(Left);
        xboxRS = _gamepadDrive.getX(Right);
        xboxLT = _gamepadDrive.getTriggerAxis(Left);
        xboxRT = _gamepadDrive.getTriggerAxis(Right);
        xboxLB = _gamepadDrive.getBumper(Left);
        xboxRB = _gamepadDrive.getBumper(Right);
        xboxStartPressed = _gamepadDrive.getStartButtonPressed();
        xboxBackPressed = _gamepadDrive.getBackButtonPressed();
        xboxXPressed = _gamepadDrive.getXButtonPressed();

        ps4L1 = _gamepadShoot.getRawButton(Constants.PS4ID.l1);
        ps4R1 = _gamepadShoot.getRawButton(Constants.PS4ID.r1);
        // ps4TouchpadPressed = _gamepadShoot.getRawButtonPressed(Constants.PS4ID.touchpad);
        ps4Triangle = _gamepadShoot.getRawButtonPressed(Constants.PS4ID.triangle);
        ps4L2 = _gamepadShoot.getRawAxis(Constants.PS4ID.l2a);
        ps4R2 = _gamepadShoot.getRawAxis(Constants.PS4ID.r2a);
        ps4L3 = _gamepadShoot.getRawAxis(Constants.PS4ID.l3h);
        ps4R3 = _gamepadShoot.getRawAxis(Constants.PS4ID.r3h);
    }

    @Override
    public void autonomousInit(){
        timer.reset();
        timer.start();
    }

    @Override
    public void autonomousPeriodic(){
        // Drive forward for 5 seconds
        if (timer.get() <= 5) {
            Drivetrain.drivePeriodic(0.5, 0);
        } else {
            Drivetrain.drivePeriodic(0, 0);
        }

        if (timer.get() > 5) {
            if (Motors.aimTurret()){ // Motors.aimTurret() aims and turns flywheel with speed 1, returns true when target locked
                Motors.loadTurret(true); // bring ball to turret
            } else if (timer.get() > 10){
                Motors.loadTurret(false); // stop load motors 
                Motors.aimOff(); // stop flywheel
            }
        }

        // drive backwards for 4 seconds
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

        safety = false;
        reverse = false;

        enableElevator = false;

        locked = false;

        tempController = false;
    }

    @Override
    public void teleopPeriodic() {
        /*** XBOX CONTROLLER CONTROLS (_gamepadDrive) ***/
        if (!tempController) {
            /* DRIVER CONTROLS */
            forward = xboxRT - xboxLT;
            turn = Constants.deadband(xboxLS);
            if(xboxStartPressed) safety = !safety;
            if(xboxBackPressed) reverse = !reverse;
            
            Drivetrain.drivePeriodic(forward, turn, safety, reverse);
            
            /* ELEVATOR CONTROLS */
            if(xboxXPressed){
                enableElevator = true;
            }
            if (enableElevator) {
                Motors.liftElevator(xboxRB, xboxLB); // RB to drop, LB to lift
            }

        } else {
            /*** TEMP CONTROLLER CONTROLS ***/
            // This is disabled by default
            // To enable, set tempController to true in teleopInit()
            // This will disable the Xbox controller input
            /*** DRIVER ***/
            forward = Constants.linScale(_gamepadTemp.getRawAxis(Constants.PS4ID.r2a)) - Constants.linScale(_gamepadTemp.getRawAxis(Constants.PS4ID.l2a));
            turn = Constants.deadband(_gamepadTemp.getRawAxis(Constants.PS4ID.l3h));

            safety = _gamepadTemp.getRawButtonPressed(Constants.PS4ID.option);
            reverse = _gamepadTemp.getRawButtonPressed(Constants.PS4ID.share);

            Drivetrain.drivePeriodic(forward, turn, safety, reverse);

            /*** ELEVATOR ***/
            enableElevator = _gamepadTemp.getRawButtonPressed(Constants.PS4ID.square);
            if (enableElevator) {
                Motors.liftElevator(_gamepadTemp.getRawButton(Constants.PS4ID.l1), _gamepadTemp.getRawButton(Constants.PS4ID.r1));
            }
        }
        /*** PS4 CONTROLLER CONTROLS (_gamepadShoot) ***/
        if(ps4TouchpadPressed) manualOverride = !manualOverride;
        intakeSpeed = Constants.expScale(Constants.linScale(ps4R2)) - Constants.expScale(Constants.linScale(ps4L2));
        /* AUTO AIM */
        if (!manualOverride) {
            if (ps4R1) {
                locked = Motors.aimTurret(); // R1 to aim and start flywheel
            } else {
                Motors.aimOff();
            }
            Motors.manualIntake(intakeSpeed);
            Motors.loadTurret((locked && ps4L1) ? true : false); // load turret if target locked and L1
        }

        /* MANUAL OVERRIDE */
        aimSpeed = ps4R3;
        if (manualOverride){
            Motors.manualAim(aimSpeed);
            if (ps4Triangle) {
                Motors.manualFly(1);
                Motors.manualLoad(1);
            } else {
                Motors.manualFly(0);
                Motors.manualLoad(0);
            }
        }
    }
}