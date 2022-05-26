// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
// comment

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final double WINCH_MAX_SPEED = 1;
  private final double INTAKE_MAX_SPEED = 1;
  private final double CONVEY_MAX_SPEED = 1;

  private Timer autoTimer = new Timer();

  private Joystick mThrustMaster;
  private Joystick1038 mLogiPad;
  private Joystick driveJoystick;
  private Joystick climbJoystick;
  private Joystick1038 intakeJoystick;
  private Joystick1038 conveyJoystick;
  private Joystick1038 winchJoystick;

  private MecanumDrive mDrive;

  private CANSparkMax driveFrontLeft;
  private CANSparkMax driveFrontRight;
  private CANSparkMax driveBackLeft;
  private CANSparkMax driveBackRight;
  private CANSparkMax intake;
  private CANSparkMax conveyFront;
  private CANSparkMax shootLeft;
  private CANSparkMax climbLeft;
  private CANSparkMax conveyBack;
  private CANSparkMax shootRight;
  private CANSparkMax climbRight;
  private CANSparkMax shootPower;
  private CANSparkMax winch;

  private RelativeEncoder climbLeftEnc;
  private RelativeEncoder climbRightEnc;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    mThrustMaster = new Joystick(0);
    mLogiPad = new Joystick1038(1);
    driveJoystick = mThrustMaster;
    climbJoystick = mThrustMaster;
    intakeJoystick = mLogiPad;
    conveyJoystick = mLogiPad;
    winchJoystick = mLogiPad;

    driveFrontLeft = new CANSparkMax(1, MotorType.kBrushless);
    driveFrontLeft.setInverted(false);
    driveFrontLeft.setIdleMode(IdleMode.kBrake);
    driveFrontLeft.burnFlash();

    driveBackLeft = new CANSparkMax(2, MotorType.kBrushless);
    driveBackLeft.setInverted(false);
    driveBackLeft.setIdleMode(IdleMode.kBrake);
    driveBackLeft.burnFlash();

    driveFrontRight = new CANSparkMax(3, MotorType.kBrushless);
    driveFrontRight.setInverted(true);
    driveFrontRight.setIdleMode(IdleMode.kBrake);
    driveFrontRight.burnFlash();

    driveBackRight = new CANSparkMax(7, MotorType.kBrushless);
    driveBackRight.setInverted(true);
    driveBackRight.setIdleMode(IdleMode.kBrake);
    driveBackRight.burnFlash();

    conveyFront = new CANSparkMax(6, MotorType.kBrushed);
    conveyFront.setInverted(false);
    conveyFront.setIdleMode(IdleMode.kCoast);
    conveyFront.burnFlash();

    conveyBack = new CANSparkMax(9, MotorType.kBrushed);
    conveyBack.setInverted(false);
    conveyBack.setIdleMode(IdleMode.kCoast);
    conveyBack.follow(conveyFront, true);
    conveyBack.burnFlash();

    shootLeft = new CANSparkMax(4, MotorType.kBrushless);
    shootLeft.restoreFactoryDefaults();
    shootLeft.setInverted(false);
    shootLeft.setIdleMode(IdleMode.kCoast);
    shootLeft.burnFlash();

    shootRight = new CANSparkMax(10, MotorType.kBrushless);
    shootRight.restoreFactoryDefaults();
    shootRight.setIdleMode(IdleMode.kCoast);
    shootRight.follow(shootLeft, true);
    shootRight.burnFlash();

    shootPower = new CANSparkMax(12, MotorType.kBrushless);
    shootPower.restoreFactoryDefaults();
    shootPower.setIdleMode(IdleMode.kCoast);
    shootPower.burnFlash();

    climbLeft = new CANSparkMax(8, MotorType.kBrushless);
    climbLeft.setInverted(false);
    climbLeft.setIdleMode(IdleMode.kBrake);
    climbLeft.burnFlash();

    climbRight = new CANSparkMax(11, MotorType.kBrushless);
    climbRight.setInverted(false);
    climbRight.setIdleMode(IdleMode.kBrake);
    climbRight.burnFlash();

    intake = new CANSparkMax(5, MotorType.kBrushed);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kCoast);
    intake.burnFlash();

    winch = new CANSparkMax(13, MotorType.kBrushed);
    winch.setInverted(false);
    winch.setIdleMode(IdleMode.kBrake);
    winch.burnFlash();

    mDrive = new MecanumDrive(driveFrontLeft, driveBackLeft, driveFrontRight, driveBackRight);

    climbLeftEnc = climbLeft.getEncoder();
    climbRightEnc = climbRight.getEncoder();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Climber Encoder", climbLeftEnc.getPosition());
    SmartDashboard.putNumber("Right Climber Encoder", climbRightEnc.getPosition());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoTimer.start();
    climbLeftEnc.setPosition(0);
    climbRightEnc.setPosition(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  
  public void autonomousPeriodic() {
    if (autoTimer.get() < 1) {
      shootLeft.set(-0.4);
      shootPower.set(0.4);
    } else if (autoTimer.get() < 4) {
      conveyFront.set(-1);
      conveyBack.set(-1);
    } else if (autoTimer.get() < 7) {
      mDrive.drivePolar(0.5, 180, 0);
      shootLeft.set(0);
      shootPower.set(0);
    } else if (autoTimer.get() < 9) {
      mDrive.drivePolar(0, 0, 0);
    } else {
      mDrive.drivePolar(0, 0, 0);
      conveyFront.stopMotor();
      conveyFront.set(0);
      shootLeft.stopMotor();
      shootLeft.set(0);
      shootPower.stopMotor();
      shootPower.set(0);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    autoTimer.stop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive();
    intake();
    convey();
    shoot();
    climb();
    winch();
  }

  private void drive() {
    mDrive.driveCartesian(Math.pow(-driveJoystick.getY(), 3), Math.pow(driveJoystick.getX(), 3), 0.5 * Math.pow(driveJoystick.getZ(), 3));
  }

  private void intake() {
    boolean intakeOn = intakeJoystick.getAButton();
    boolean intakeOnReverse = intakeJoystick.getBButton();

    if (intakeOn) {
      intake.set(INTAKE_MAX_SPEED);
    } else if (intakeOnReverse) {
      intake.set(-INTAKE_MAX_SPEED);
    } else {
      intake.stopMotor();
      intake.set(0);
    }
  }

  private void convey() {
    boolean conveyOn = conveyJoystick.getXButton();
    boolean conveyOnReverse = conveyJoystick.getYButton();

    if (conveyOn) {
      conveyFront.set(-CONVEY_MAX_SPEED);
    } else if (conveyOnReverse) {
      conveyFront.set(CONVEY_MAX_SPEED);
    } else {
      conveyFront.stopMotor();
      conveyFront.set(0);
    }
  }

  private void setShooterSpeed(double left, double power) {
    shootLeft.set(left);
    shootPower.set(power);
  }

  private void shoot() {
    if (mLogiPad.getLeftJoystickClick() && mLogiPad.getRightJoystickClick()) {
      shootLeft.stopMotor();
      shootPower.stopMotor();
      shootLeft.set(0);
      shootPower.set(0);
    } else if (mLogiPad.getLeftJoystickClick()) {
      setShooterSpeed(-0.4, 0.4);
    } else if (mLogiPad.getRightJoystickClick()) {
      setShooterSpeed(-0.8, 0.8);
    } else {
      shootLeft.stopMotor();
      shootPower.stopMotor();
      setShooterSpeed(0, 0);
    }

  }

  private void climbLeft() {
    if (climbJoystick.getRawButton(5)) {
      climbLeft.set(-1);
    } else if (climbJoystick.getRawButton(10)) {
      climbLeft.set(1);
    } else if (climbJoystick.getRawButton(6)) {
      climbLeft.set(-1);
    } else if (climbJoystick.getRawButton(9)) {
      climbLeft.set(1);
    } else {
      climbLeft.stopMotor();
      climbLeft.set(0);
    }
  }

  private void climbRight() {
    if (climbJoystick.getRawButton(7)) {
      climbRight.set(1);
    } else if (climbJoystick.getRawButton(8)) {
      climbRight.set(-1);
    } else if (climbJoystick.getRawButton(6)) {
      climbRight.set(1);
    } else if (climbJoystick.getRawButton(9)) {
      climbRight.set(-1);
    } else {
      climbRight.stopMotor();
      climbRight.set(0);
    }
  }

  private void climb() {
    climbLeft();
    climbRight();
  }

  private void winch() {
    if (winchJoystick.getLeftBumper()) {
      winch.set(WINCH_MAX_SPEED);
    } else if (mLogiPad.getRightBumper()) {
      winch.set(-WINCH_MAX_SPEED);
    } else {
      winch.stopMotor();
      winch.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
