package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;

/*
Required Dependencies
wpilib API
http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json
http://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
http://revrobotics.com/content/sw/color-sensor-v3/sdk/REVColorSensorV3.json
*/

// PS4 Button mapping can be found in ps4_buttons.txt

public class Robot extends TimedRobot {
  //PDP
  PowerDistributionPanel pdp = new PowerDistributionPanel();
  double current[] = new double[16];

  /** Drive Motor Controllers */
  CANSparkMax leftMaster = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftSide = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightSide = new CANSparkMax(2, MotorType.kBrushless);

  SpeedControllerGroup leftGrouping = new SpeedControllerGroup(leftMaster, leftSide);
  SpeedControllerGroup rightGrouping = new SpeedControllerGroup(rightMaster, rightSide);

  DifferentialDrive Drive = new DifferentialDrive(leftGrouping, rightGrouping);

  /** Other Motor Controllers */
  TalonSRX temp = new TalonSRX(5);
  VictorSPX Shooter = new VictorSPX(6); // controls turret launch motor
  Victor Aim = new Victor(7); // controls turret aim motor
  VictorSPX IntakeWheel = new VictorSPX(8); // controls the intake wheels
  VictorSPX IntakeBelt = new VictorSPX(9); // controls the intake elevator motor
  VictorSPX IntakeUpandDown = new VictorSPX(10); // controls the raising/lowering of intake bar itself
  VictorSPX FortuneUpandDown = new VictorSPX(11); // currently unused in code
  VictorSPX toShoot = new VictorSPX(12); // brings the POWERCELL up to the actual firing mechanism
  VictorSPX elevator = new VictorSPX(13); // elevator system, used for two motors (one controller)

  /** Gamepad */
  XboxController _gamepadDrive = new XboxController(0); // driving
  Joystick _gamepadShoot = new Joystick(1); // turret control

  /** Vision / Raspberry Pi */
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("TurretCam");
  NetworkTableEntry targetX;
  NetworkTableEntry poseArray; // 3D positioning [x distance meters, y distance meters, angle degrees]
  NetworkTableEntry isValid;

  boolean targetFound; // if a target was found
  double rotationOffset; // Current offset
  double angleTolerance = 5; // Deadzone for alignment
  double poseX; // X distance in meters
  double poseY; // Y distance in meters
  double aimDist; // distance magnitude in meters (pythagorean)

  // autonomous
  int count = 0;
  double autoForward = 0.25;
  double autoTurn = 0;
  boolean canTurn = false;
  double nowAngle = 0;
  boolean isPneumatic = false;
  double pneuCount = 0;

  // Gyro
  ADXRS450_Gyro gyroBoy = new ADXRS450_Gyro();
  double angle = 0;
  double rate = 0;
  boolean gyroConnected = false;

  // speed divider
  double speedDiv = 4.0;

  // ramping
  double prevVal = 0;

  // peripherals
  Relay lights = new Relay(0);
  Relay.Value lightsOn = Relay.Value.kForward;
  Relay.Value lightsOff = Relay.Value.kOff;

  // hatch light
  DigitalOutput hatchLight = new DigitalOutput(9);

  // Control Mapping for PS4
  boolean circle, square, cross, triangle, up, down, left, right, l1, l3, r1, r3, option, share, l2b, r2b, touchpad, psbutton;
  double r3h, r3v, l3h, l3v, r2a, l2a;
  int pov;

  // controls
  boolean reverseControls = false;
  double reverseControlDelay = 1;
  boolean reverse = false;

  boolean aiming = false;
  double shootSpeed;
  double intakeSpeed;
  boolean intakeMove;
  double manualAim;
  boolean toFire;
  boolean manualOverride = false;

  double forward;
  double turn;

  boolean eleUp;
  boolean eleSet;

  Timer timer = new Timer();

  // safety
  boolean safety = false;

  /** Limit Switches */
  DigitalInput limitSwitchUpper = new DigitalInput(0);
  DigitalInput limitSwitchLower = new DigitalInput(1);

  /** IR Sensor (intake) */
  DigitalInput intakeIR = new DigitalInput(2);
  /* *****************ROBOT INIT***************** */
  @Override
  public void robotInit() {
    gyroBoy.calibrate();

    lights.set(lightsOn);

    /* Configure output direction */
    leftMaster.setInverted(false);
    leftSide.setInverted(false);
    rightMaster.setInverted(false);
    rightSide.setInverted(false);

    // turns lights off
    lights.set(lightsOn);

    // Get initial values (Vision)
    targetX = cameraTable.getEntry("yaw");
    poseArray = cameraTable.getEntry("targetPose");
    isValid = cameraTable.getEntry("isValid");
  }

  /* ROBOT PERIODIC */
  @Override
  public void robotPeriodic() {
    for (int i = 0; i < current.length; i++) {
      current[i] = pdp.getCurrent(i);
    }

    /* Vision stuff */
    rotationOffset = targetX.getDouble(0.0);
    poseX = poseArray.getDoubleArray(new double[] { 0.0, 0.0, 0.0 })[0];
    poseY = poseArray.getDoubleArray(new double[] { 0.0, 0.0, 0.0 })[1];

    aimDist = Math.sqrt(Math.pow(poseX, 2) + Math.pow(poseY, 2));
    targetFound = isValid.getBoolean(false);

    SmartDashboard.putNumber("Aim Yaw: ", rotationOffset);
    SmartDashboard.putNumber("Aim Distance: ", aimDist);
    SmartDashboard.putBoolean("Target Found: ", targetFound);

    // PS4 Controls
    cross = _gamepadShoot.getRawButton(2);
    circle = _gamepadShoot.getRawButton(3);
    square = _gamepadShoot.getRawButton(1);
    triangle = _gamepadShoot.getRawButton(4);

    r1 = _gamepadShoot.getRawButton(6);
    l1 = _gamepadShoot.getRawButton(5);

    r2b = _gamepadShoot.getRawButton(8);
    l2b = _gamepadShoot.getRawButton(7);

    r3 = _gamepadShoot.getRawButton(12);
    l3 = _gamepadShoot.getRawButton(11);

    pov = _gamepadShoot.getPOV(); // dpad

    option = _gamepadShoot.getRawButton(10);
    share = _gamepadShoot.getRawButton(9);
    psbutton = _gamepadShoot.getRawButton(13);
    touchpad = _gamepadShoot.getRawButton(14);

    r3h = _gamepadShoot.getRawAxis(2); // r3 horizontal
    r3v = _gamepadShoot.getRawAxis(5);
    l3v = _gamepadShoot.getRawAxis(1);
    l3h = _gamepadShoot.getRawAxis(0);
    r2a = _gamepadShoot.getRawAxis(4);
    l2a = _gamepadShoot.getRawAxis(3);
  }

  /* *****************AUTO INIT***************** */
  @Override
  public void autonomousInit() {
    // timer.reset();
  }

  /* *****************AUTO PERIODIC***************** */
  @Override
  public void autonomousPeriodic() {
    // Drive.arcadeDrive(1.0, 0);
  }

  /* *****************TELEOP INIT***************** */
  @Override
  public void teleopInit() {
    /** Light on */
    hatchLight.set(true);

    /** Drive initialization */
    forward = 0;
    turn = 0;

    // turret controls
    manualOverride = false;
  }

  /* *****************TELEOP PERIODIC***************** */
  @Override
  public void teleopPeriodic() {
    /** gyroscope */
    angle = gyroBoy.getAngle();
    rate = gyroBoy.getRate();
    gyroConnected = gyroBoy.isConnected();

    if (angle > 360) {
      int angleNum = (int) angle % 360;
      angle -= 360 * angleNum;
    }

    if (angle < 0) {
      int angleNum = (int) angle % -360;
      angle += 360 * (angleNum + 1);
    }

    /******************************
     * Driver Controller (_gamepadDrive)
     ******************************/
    /** Gamepad Drive processing */
    // forward is RT axis minus LT axis (scaled to [-1, 1])
    forward = triggers(_gamepadDrive.getTriggerAxis(GenericHID.Hand.kRight)) - triggers(_gamepadDrive.getTriggerAxis(GenericHID.Hand.kLeft));
    turn = _gamepadDrive.getX(GenericHID.Hand.kLeft);

    // limit the acceleration / decceleration
    if (forward > 0) {
      if ((forward - prevVal) >= 0.07) {
        forward = prevVal + 0.07;
      }
    } else {
      if ((forward - prevVal) <= -0.07) {
        forward = prevVal - 0.07;
      }
    }

    prevVal = forward;

    if (!reverseControls) {
      forward *= -1;
    }

    turn = Scale(Deadband(turn));

    /** Arcade Drive */
    Drive.arcadeDrive(forward, turn);

    if (safety) {
      forward /= speedDiv;
      turn /= speedDiv;
    }

    /** check safety mode */
    if (_gamepadDrive.getStartButtonPressed()) // start button toggles safety
      safety = !safety;

    /** reverse button */
    if (_gamepadDrive.getBackButton()) // reverse controls as back button is pressed
      reverse = !reverse;

    /** TODO Elevator */
    // RB = elevator up
    // LB = elevator set
    eleUp = _gamepadDrive.getBumper(GenericHID.Hand.kRight);
    eleSet = _gamepadDrive.getBumper(GenericHID.Hand.kLeft);
    
    if(eleUp && !eleSet){
      elevator.set(ControlMode.PercentOutput, 0.5);
    } else {
      elevator.set(ControlMode.PercentOutput, 0.0);
    }

    if(eleSet && !eleUp){
      elevator.set(ControlMode.PercentOutput, -1.0);
    } else {
      elevator.set(ControlMode.PercentOutput, 0.0);
    }

    /******************************
     * Shooter Controler (_gamepadShoot)
     ******************************/
    /** Shooting */
    aiming = l2b; // L2 as a button
    toFire = r1; // R1 button

    shootSpeed = l2a; // L2 analog
    manualAim = r3h; // Right stick analog

    //override toggle
    if(_gamepadShoot.getRawButtonPressed(14)){ //touchpad
      manualOverride = !manualOverride;
    }

    if (targetFound && aiming && !manualOverride) {
      Shooter.set(ControlMode.PercentOutput, getShootSpeed(aimDist)); // auto aim and set speed (ideally)
      if (rotationOffset > angleTolerance) {
        Aim.set(-0.5); // TODO placeholder 50% power, figure out optimal value
      } else if (rotationOffset < -angleTolerance) {
        Aim.set(0.5);
      }
    }

    if (Deadband(manualAim) != 0) {
      if(manualAim > 0){
        Aim.set(manualAim > 0.5 ? 0.5 : manualAim);
      } else {
        Aim.set(manualAim < -0.5 ? -0.5 : manualAim);
      }
    }
    //TODO manual speed

    // bring the POWERCELL up to the firing mech
    if (toFire) {
      toShoot.set(ControlMode.PercentOutput, 1.0);
    } else {
      toShoot.set(ControlMode.PercentOutput, 0.0);
    }

    /** Intake */
    intakeSpeed = r2a - l2a; // R2 and L2 analog stick
    IntakeWheel.set(ControlMode.PercentOutput, intakeSpeed);
    IntakeBelt.set(ControlMode.PercentOutput, intakeIR.get() ? 1.0 : 0.0); // Set belt speed to 1 if ball is detected, otherwise stop

    if (_gamepadShoot.getRawButtonPressed(6)) {
      IntakeUpandDown.set(ControlMode.PercentOutput, intakeMove ? 1.0 : -1.0); // TODO determine up/down speeds
    }

    if (limitSwitchUpper.get() || limitSwitchLower.get()) { // if limit switch is hit, set speed 0
      IntakeUpandDown.set(ControlMode.PercentOutput, 0.0);
      intakeMove = !intakeMove; // toggle intakeMove to control raise/lower
    }

    /** Control Panel */
    if (cross) {

    }

    if (circle) {

    }

    if (triangle) {

    }

    if (square) {

    }

    /** Smart Dashboard */
    SmartDashboard.putNumber("Angle: ", angle);
    SmartDashboard.putNumber("Rate: ", rate);
    SmartDashboard.putBoolean("gyro Connected: ", gyroConnected);

    SmartDashboard.putBoolean("Safety: ", safety);
    SmartDashboard.putBoolean("Reverse: ", reverseControls);
    SmartDashboard.putBoolean("Manual aim: ", manualOverride);
  }

  /** Deadband 3 percent, used on the gamepad */
  double Deadband(double value) {
    /* Upper deadband */
    if (value >= +0.02)
      return value;

    /* Lower deadband */
    if (value <= -0.02)
      return value;

    /* Outside deadband */
    return 0;
  }

  /**
   * this method scales the joystick output so that the robot moves slower when
   * the joystick is barely moved, but allows for full power
   */
  double Scale(double value) {
    value *= -1;
    if (value >= -0.9 && value <= 0.9) {
      if (value > 0) {
        value = Math.pow(value, 2);
      } else {
        value = Math.pow(value, 2);
        value *= -1;
      }
    }
    return value;
  }

  // Scales trigger values from [-1, 1] to [0, 1] (default -1 unpressed)
  double triggers(double value) {
    return (value * 0.5) + 0.5;
  }

  // TODO find relationship between percent power of shooterspeed and distance
  // Math: v(d)=sqrt((-10d^2)/(1.5-d)), assuming the angle of velocity is 45deg
  // from horizontal
  // Should add more leniency because real world and potential inaccurate distance
  // measure
  double getShootSpeed(double distance) {
    double percent = 0;
    return percent;
  }

}
