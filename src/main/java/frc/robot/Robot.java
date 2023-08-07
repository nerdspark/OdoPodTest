// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
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
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final CANSparkMax motor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder motorEncoder = motor.getEncoder();
  private final Encoder encoderLeft = new Encoder(0, 1);
  private final Encoder encoderRight = new Encoder(2, 3);
  // private final DigitalInput encoderLeft = new DigitalInput(0);
  // private final PWM encoderRight = new PWM(1); 
  private double motorPosition = 0;
  private double motorVelocity = 0;
  private double motorTargetVelocity = 0;
  private boolean goingIn = false;
  private double leftEncoderStart = 0;
  private double rightEncoderStart = 0;
  private double motorStart = 0;
  private double runCount = 0;
  private double prevRampRate = Constants.rampRate;
  private boolean stopped, stopGraph = false;
  private ShuffleboardTab tab = Shuffleboard.getTab("main");
  private GenericEntry start = tab.add("go BUTTON", false).getEntry();
  private GenericEntry got = tab.add("got", true).getEntry();
  // private GenericEntry goinIn = tab.add("goin in", true).getEntry();
  private GenericEntry motorCurrent = tab.add("motorCurrent", 0).getEntry();
  private GenericEntry resetMotor = tab.add("reset motor BUTTON", false).getEntry();
  // private GenericEntry motorPos = tab.add("Motor Position", 0).getEntry();
  // private GenericEntry motorVel = tab.add("Motor Velocity", 0).getEntry();
  private GenericEntry maxSpeed = tab.add("Max Speed INPUT", 1).getEntry();
  private GenericEntry odoAngle = tab.add("Odo Angle INPUT", 0).getEntry();
  private GenericEntry odoMultiplier = tab.add("Odo Multiplier INPUT", 1).getEntry();

  private GenericEntry motorDisplacementCalc = tab.add("Motor Displacement Calculation", 0).getEntry();
  private GenericEntry leftWheelDisplacement = tab.add("Left Wheel Displacement Calculation", 0).getEntry();
  private GenericEntry rightWheelDisplacement = tab.add("Right Wheel Displacement Calculation", 0).getEntry();
  private GenericEntry totalWheelDisplacement = tab.add("Total odometry displacement calculation", 0).getEntry();

  private GenericEntry resetEncoders = tab.add("Reset Encoders for graph BUTTON", false).getEntry();
  private GenericEntry rampRate = tab.add("Motor Ramp Rate INPUT", Constants.rampRate).getEntry();
  private GenericEntry stopDist = tab.add("Deaccel Distance INPUT", Constants.inTolerance).getEntry();

  private Joystick joystick = new Joystick(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
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
    
    // SmartDashboard.putNumber("Motor Position", motorPosition);
    // SmartDashboard.putNumber("Motor Velocity", motorVelocity);
    // SmartDashboard.putNumber("Motor Velocity Target", motorTargetVelocity);
    // SmartDashboard.putNumber("EncoderLeft Ticks", encoderLeft.get());
    // SmartDashboard.putNumber("EncoderRight Ticks", encoderRight.get());
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
    // m_autoSelected = m_chooser.getSelected();
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
    motor.setOpenLoopRampRate(rampRate.getDouble(Constants.rampRate));
    motor.setIdleMode(IdleMode.kBrake);
    // motorEncoder.setPosition(0);
    motor.setInverted(Constants.motorInverted);
    motorEncoder.setPositionConversionFactor(Constants.NeoTickstoFeet);
    motorEncoder.setVelocityConversionFactor(Constants.NeoTickstoFeet/60);
    goingIn = false;
    // encoderLeft.setDistancePerPulse(Constants.encoderTickstoFeet);
    // encoderRight.setDistancePerPulse(Constants.encoderTickstoFeet);
    resetMotor.setBoolean(true);
    start.setBoolean(false);
    stopGraph = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (rampRate.getDouble(Constants.rampRate) != prevRampRate) {
      motor.setOpenLoopRampRate(rampRate.getDouble(Constants.rampRate));
    }
    prevRampRate = rampRate.getDouble(Constants.rampRate);
    double multiplier = odoMultiplier.getDouble(1);
    boolean tart = start.getBoolean(false);
    double OdoAngle = odoAngle.getDouble(0);
    motorPosition = motorEncoder.getPosition();
    motorVelocity = motorEncoder.getVelocity();

    if (resetEncoders.getBoolean(false) || joystick.getRawButton(3)) {
          leftEncoderStart = encoderLeft.get();
          rightEncoderStart = encoderRight.get();
          motorStart = motorEncoder.getPosition();
    }

    if (motorPosition > Constants.travelDist - Constants.outTolerance) {
      if (tart) {
        // if (!goingIn) {
          // leftEncoderStart = encoderLeft.get();
          // rightEncoderStart = encoderRight.get();
          // motorStart = motorEncoder.getPosition();
        // }
        goingIn = true;
      } else if (!goingIn) {
        stopped = true;
      }
    }
    if (motorPosition < stopDist.getDouble(Constants.inTolerance)) {
      if (goingIn) {
        runCount +=1;
      }
      goingIn = false;
    } 
    stopped = Math.abs(joystick.getRawAxis(0)) > 0.1 ? stopped : (tart ? false : stopped);

    if (resetMotor.getBoolean(false)){
      resetMotor.setBoolean(motor.getOutputCurrent() < 2);
      motor.set(Constants.speedOut);
      if (motor.getOutputCurrent() > 2) {
        motorEncoder.setPosition(Constants.travelDist);
      }
    } else {
      if (stopped) {
        motorTargetVelocity = 0;
      } else {
        if (goingIn) {
          motorTargetVelocity = -maxSpeed.getDouble(1);
        } else if (motorVelocity < -0.1) {
          motorTargetVelocity = 0;
        } else {
          motorTargetVelocity = Constants.speedOut;
        }
      }
      if (Math.abs(joystick.getRawAxis(0)) > 0.1) {
        motorTargetVelocity = Math.abs(joystick.getRawAxis(0))*joystick.getRawAxis(0)/2.5;
      }
      motor.set(motorTargetVelocity);

    }
    double encoderLeftDivisor = Math.cos((OdoAngle - 45) * Math.PI/180);
    double encoderRightDivisor = Math.cos((OdoAngle - 135) * Math.PI/180);
    double encoderLeftDisplacement = (encoderLeft.get() - leftEncoderStart) * Constants.encoderTickstoFeet * multiplier * encoderLeftDivisor;
    double encoderRightDisplacement = (encoderRight.get() - rightEncoderStart) * Constants.encoderTickstoFeet * multiplier * encoderRightDivisor;
    double motorDisplacement = -1 * (motorEncoder.getPosition() - motorStart);
    double encodersDisplacement = Math.abs(encoderLeftDivisor) <= 0.1 ? encoderRightDisplacement : (Math.abs(encoderRightDivisor) <= 0.1 ? encoderLeftDisplacement : ((encoderLeftDisplacement + encoderRightDisplacement) / 2));

    // SmartDashboard.putNumber("Motor Position", motorPosition);
    SmartDashboard.putNumber("Motor Velocity", motorVelocity);
    // SmartDashboard.putNumber("Motor Velocity Target", motorTargetVelocity);
    // SmartDashboard.putNumber("EncoderLeft Position ticks", encoderLeft.getDistance());
    // SmartDashboard.putNumber("EncoderRight Position ticks", encoderRight.getDistance());
    // SmartDashboard.putNumber("Calculated Position feet", encodersDisplacement);
    SmartDashboard.putNumber("Run Count", runCount);

    // SmartDashboard.putNumber("encoderleftdivisor", encoderLeftDivisor);
    // SmartDashboard.putNumber("encoderleftstart", leftEncoderStart);
    // SmartDashboard.putNumber("leftencoderdisplacement calculation", encoderLeftDisplacement);
    // SmartDashboard.putNumber("rightencoderdisplacement calculation", encoderRightDisplacement);
    // SmartDashboard.putNumber("motordisplacement calculation", motorDisplacement);

    got.setBoolean(tart);
    // goinIn.setBoolean(goingIn);

    motorCurrent.setDouble(motor.getOutputCurrent());
    // motorPos.setDouble(motorPosition);
    // motorVel.setDouble(motorVelocity);
    if (!stopGraph) {
      motorDisplacementCalc.setDouble(motorDisplacement);
      leftWheelDisplacement.setDouble(encoderLeftDisplacement);
      rightWheelDisplacement.setDouble(encoderRightDisplacement);
      totalWheelDisplacement.setDouble(encodersDisplacement);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    stopGraph = true;
  }

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
