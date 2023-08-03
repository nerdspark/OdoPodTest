// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final CANSparkMax motor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder motorEncoder = motor.getEncoder();
  private final PWM encoderLeft = new PWM(0); 
  private final PWM encoderRight = new PWM(1); 
  private double motorPosition = 0;
  private double motorVelocity = 0;
  private double motorTargetVelocity = 0;
  private boolean goingIn = false;
  private double leftEncoderStart = 0;
  private double rightEncoderStart = 0;
  private double runCount = 0;
  private boolean stopped = false;
  private ShuffleboardTab tab = Shuffleboard.getTab("main");
  private GenericEntry start = tab.add("go", false).getEntry();
  private GenericEntry got = tab.add("got", true).getEntry();
  private GenericEntry goinIn = tab.add("goin in", true).getEntry();
  private GenericEntry motorCurrent = tab.add("motorCurrent", 0).getEntry();
  private GenericEntry resetMotor = tab.add("reset motor button", true).getEntry();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    SmartDashboard.putNumber("Motor Position", motorPosition);
    SmartDashboard.putNumber("Motor Velocity", motorVelocity);
    SmartDashboard.putNumber("Motor Velocity Target", motorTargetVelocity);
    SmartDashboard.putNumber("EncoderLeft Position", encoderLeft.getPosition());
    SmartDashboard.putNumber("EncoderRight Position", encoderRight.getPosition());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    motor.setOpenLoopRampRate(Constants.rampRate);
    motor.setIdleMode(IdleMode.kBrake);
    motorEncoder.setPosition(0);
    motor.setInverted(Constants.motorInverted);
    motorEncoder.setPositionConversionFactor(Constants.NeoTickstoFeet);
    motorEncoder.setVelocityConversionFactor(Constants.NeoTickstoFeet/60);
    goingIn = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    boolean tart = start.getBoolean(false);
    double OdoAngle = SmartDashboard.getNumber("Pod Angle - Y fwd, right pos, degrees", 0);
    motorPosition = motorEncoder.getPosition();
    motorVelocity = motorEncoder.getVelocity();

    if (motorPosition > Constants.travelDist - Constants.outTolerance) {
      if (tart) {
        if (!goingIn) {
          leftEncoderStart = encoderLeft.getPosition();
          rightEncoderStart = encoderRight.getPosition();
        }
        goingIn = true;
      } else if (!goingIn) {
        stopped = true;
      }
    }
    if (motorPosition < Constants.inTolerance) {
      if (goingIn) {
        runCount +=1;
      }
      goingIn = false;
    }
    stopped = tart ? false : stopped;

    if (resetMotor.getBoolean(true)){
      resetMotor.setBoolean(motor.getOutputCurrent() < 5);
      motor.set(-Constants.speedOut);
      if (motor.getOutputCurrent() > 5) {
        motorEncoder.setPosition(0);
      }
    } else {
      if (stopped) {
        motorTargetVelocity = 0;
      } else {
        if (goingIn) {
          motorTargetVelocity = -Constants.maxSpeed;
        } else if (motorVelocity < -0.1) {
          motorTargetVelocity = 0;
        } else {
          motorTargetVelocity = Constants.speedOut;
        }
      }
      motor.set(motorTargetVelocity);
    }
    double encoderLeftDivisor = Math.cos((OdoAngle - 45) * Math.PI/180);
    double encoderRightDivisor = Math.cos((OdoAngle - 45) * Math.PI/180);
    double encoderLeftDisplacement = (encoderLeft.getPosition() - leftEncoderStart) * Constants.encoderTickstoFeet / encoderLeftDivisor;
    double encoderRightDisplacement = (encoderRight.getPosition() - rightEncoderStart) * Constants.encoderTickstoFeet / encoderLeftDivisor;
    double encodersDisplacement = Math.abs(encoderLeftDivisor) <= 0.1 ? encoderRightDisplacement : (Math.abs(encoderRightDivisor) <= 0.1 ? encoderLeftDisplacement : (encoderLeftDisplacement + encoderRightDisplacement) / 2);

    SmartDashboard.putNumber("Motor Position", motorPosition);
    SmartDashboard.putNumber("Motor Velocity", motorVelocity);
    SmartDashboard.putNumber("Motor Velocity Target", motorTargetVelocity);
    SmartDashboard.putNumber("EncoderLeft Position ticks", encoderLeft.getPosition());
    SmartDashboard.putNumber("EncoderRight Position ticks", encoderRight.getPosition());
    SmartDashboard.putNumber("Calculated Position feet", encodersDisplacement);
    SmartDashboard.putNumber("Run Count", runCount);
    got.setBoolean(tart);
    goinIn.setBoolean(goingIn);

    motorCurrent.setDouble(motor.getOutputCurrent());
  }

  // public boolean resetMotor() {
  //   if (Math.abs(motor.getOutputCurrent()) < 5) {
  //     // motor.set(0.05);
  //     return true;
  //   } else {
  //     motorEncoder.setPosition(0);
  //     return false;
  //   }
  // }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
