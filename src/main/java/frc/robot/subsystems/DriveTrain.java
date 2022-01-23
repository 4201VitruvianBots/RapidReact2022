// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.Constants.DriveTrain.MotorPosition;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;

/** A differential drivetrain with two falcon motors on each side */
public class DriveTrain extends SubsystemBase {

  // PID constants for the drive motors
  private final double kP = 1.2532;
  private final double kI = 0;
  private final double kD = 0;

  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveTrain.kTrackWidthMeters);
  private final DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveTrain.ksVolts, Constants.DriveTrain.kvVoltSecondsPerMeter, Constants.DriveTrain.kaVoltSecondsSquaredPerMeter);

  PIDController leftPIDController = new PIDController(kP, kI, kD);
  PIDController rightPIDController = new PIDController(kP, kI, kD);

  private final HashMap<MotorPosition,TalonFX> driveMotors = new HashMap<MotorPosition,TalonFX>(Map.of(
    MotorPosition.LEFT_FRONT, new TalonFX(Constants.DriveTrain.leftFrontDriveMotor),
    MotorPosition.LEFT_REAR, new TalonFX(Constants.DriveTrain.leftRearDriveMotor),
    MotorPosition.RIGHT_FRONT, new TalonFX(Constants.DriveTrain.rightFrontDriveMotor),
    MotorPosition.RIGHT_REAR, new TalonFX(Constants.DriveTrain.rightRearDriveMotor)
  ));

  // To hold on to the values to set the simulation motors
  double m_leftOutput, m_rightOutput;

  private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

  /** Simulation Gyro */
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private DriveTrainNeutralMode neutralMode = DriveTrainNeutralMode.COAST;

  private DifferentialDrivetrainSim m_drivetrainSimulator;

  /** A differential drivetrain with two falcon motors on each side */
  public DriveTrain() {
    // Set up DriveTrain motors
    configureCtreMotors();

    navX.reset();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeadingDegrees()));

    if (RobotBase.isSimulation()) {
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              Constants.DriveTrain.kDrivetrainPlant,
              Constants.DriveTrain.kDriveGearbox,
              Constants.DriveTrain.kDriveGearing,
              Constants.DriveTrain.kTrackWidthMeters,
              Constants.DriveTrain.kWheelDiameterMeters / 2.0,
              null); // VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
    }
    // To display current commands on SmartDashboard
    SmartDashboard.putData("DT Subsystem", this);
  }

  private void configureCtreMotors() {
    for (TalonFX motor : driveMotors.values()) {
      motor.configFactoryDefault();

      motor.configOpenloopRamp(0.1);
      motor.configClosedloopRamp(0.1);
      motor.setNeutralMode(NeutralMode.Coast);
      motor.configForwardSoftLimitEnable(false);
      motor.configReverseSoftLimitEnable(false);

      
      motor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 30, 0, 0));

      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    driveMotors.get(MotorPosition.LEFT_FRONT).setInverted(false);
    driveMotors.get(MotorPosition.LEFT_REAR).setInverted(false);
    driveMotors.get(MotorPosition.RIGHT_FRONT).setInverted(true);
    driveMotors.get(MotorPosition.RIGHT_REAR).setInverted(true);

    driveMotors.get(MotorPosition.LEFT_FRONT).setSensorPhase(false);
    driveMotors.get(MotorPosition.RIGHT_FRONT).setSensorPhase(false);

    driveMotors.get(MotorPosition.LEFT_REAR).set(ControlMode.Follower, driveMotors.get(MotorPosition.LEFT_FRONT).getDeviceID());
    driveMotors.get(MotorPosition.RIGHT_REAR).set(ControlMode.Follower, driveMotors.get(MotorPosition.RIGHT_FRONT).getDeviceID());
    driveMotors.get(MotorPosition.LEFT_REAR).setNeutralMode(NeutralMode.Brake);
    driveMotors.get(MotorPosition.RIGHT_REAR).setNeutralMode(NeutralMode.Brake);

    driveMotors.get(MotorPosition.LEFT_REAR).configOpenloopRamp(0);
    driveMotors.get(MotorPosition.RIGHT_REAR).configOpenloopRamp(0);
  }

  /** Gets the number of counts made by a specified encoder */
  public double getEncoderCount(MotorPosition position) {
    return driveMotors.get(position).getSelectedSensorPosition();
  }
  
  /** Clockwise negative */
  public double getHeadingDegrees() {
    if (RobotBase.isReal()) return Math.IEEEremainder(-navX.getAngle(), 360);
    else
      return Math.IEEEremainder(m_gyro.getAngle(), 360)
          * (Constants.DriveTrain.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetAngle() {
    navX.zeroYaw();
  }
  
  public void setNavXOffsetDegrees(double angle) {
    navX.setAngleAdjustment(angle);
  }

  public double getWheelDistanceMeters(MotorPosition position) {
    return driveMotors.get(position).getSelectedSensorPosition()
      * Constants.DriveTrain.kEncoderDistancePerPulseMeters;
  }

  public double getMotorInputCurrentAmps(MotorPosition position) {
    return driveMotors.get(position).getSupplyCurrent();
  }
  
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  public void resetEncoderCounts() {
    driveMotors.get(MotorPosition.LEFT_FRONT).setSelectedSensorPosition(0);
    driveMotors.get(MotorPosition.RIGHT_FRONT).setSelectedSensorPosition(0);
  }

  /** Takes values in percent output, will be reduced proportionally so the max is still 1.0 */
  public void setMotorArcadeDrive(double throttle, double turn) {
    double leftPWM = throttle + turn;
    double rightPWM = throttle - turn;

    // Normalization
    double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
    if (magnitude > 1.0) {
      leftPWM /= magnitude;
      rightPWM /= magnitude;
    }

    setMotorPercentOutput(leftPWM, rightPWM);
    // setMotorVelocityMetersPerSecond(leftPWM * Constants.DriveTrain.kMaxVelocityMetersPerSecond,
    // rightPWM * Constants.DriveTrain.kMaxVelocityMetersPerSecond);
  }

  /** Input is in percent output */
  public void setMotorTankDrive(double leftOutput, double rightOutput) {
    setMotorVelocityMetersPerSecond(
        leftOutput * Constants.DriveTrain.kMaxVelocityMetersPerSecond,
        rightOutput * Constants.DriveTrain.kMaxVelocityMetersPerSecond);
  }

  public void setVoltageOutput(double leftVoltage, double rightVoltage) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage)) > batteryVoltage) {
      leftVoltage *= batteryVoltage / 12.0;
      rightVoltage *= batteryVoltage / 12.0;
    }
    SmartDashboardTab.putNumber("DriveTrain", "Left Voltage", leftVoltage);
    SmartDashboardTab.putNumber("DriveTrain", "Right Voltage", rightVoltage);

    setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);
  }

  private void setMotorPercentOutput(double leftOutput, double rightOutput) {
    m_leftOutput = leftOutput;
    m_rightOutput = rightOutput;
    driveMotors.get(MotorPosition.LEFT_FRONT).set(ControlMode.PercentOutput, leftOutput);
    driveMotors.get(MotorPosition.RIGHT_FRONT).set(ControlMode.PercentOutput, rightOutput);
  }

  /** Sets the motor velocities and calculates feedforward */
  public void setMotorVelocityMetersPerSecond(double leftSpeed, double rightSpeed) {
    m_leftOutput = leftSpeed / Constants.DriveTrain.kMaxVelocityMetersPerSecond;
    m_rightOutput = rightSpeed / Constants.DriveTrain.kMaxVelocityMetersPerSecond;
    driveMotors.get(MotorPosition.LEFT_FRONT).set(
        ControlMode.Velocity,
        m_leftOutput / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10),
        DemandType.ArbitraryFeedForward,
        feedforward.calculate(m_leftOutput));
    driveMotors.get(MotorPosition.RIGHT_FRONT).set(
        ControlMode.Velocity,
        m_rightOutput / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10),
        DemandType.ArbitraryFeedForward,
        feedforward.calculate(m_rightOutput));
  }

  /**
   * Sets drivetrain motors to coast/brake
   *
   * @param mode Which mode (all brake, all coast, or half coast and half brake)
   */
  public void setDriveTrainNeutralMode(DriveTrainNeutralMode mode) {
    neutralMode = mode;
    switch (mode) {
      case COAST:
        for (TalonFX motor : driveMotors.values()) motor.setNeutralMode(NeutralMode.Coast);
        break;
      case BRAKE:
        for (TalonFX motor : driveMotors.values()) motor.setNeutralMode(NeutralMode.Brake);
        break;
      case HALF_BRAKE:
      default:
        driveMotors.get(MotorPosition.LEFT_FRONT).setNeutralMode(NeutralMode.Brake);
        driveMotors.get(MotorPosition.LEFT_REAR).setNeutralMode(NeutralMode.Coast);
        driveMotors.get(MotorPosition.RIGHT_FRONT).setNeutralMode(NeutralMode.Brake);
        driveMotors.get(MotorPosition.RIGHT_REAR).setNeutralMode(NeutralMode.Coast);
        break;
    }
  }

  public DifferentialDriveWheelSpeeds getSpeedsMetersPerSecond() {
    double leftMetersPerSecond = 0, rightMetersPerSecond = 0;

      // getSelectedSensorVelocity() returns values in units per 100ms. Need to convert value to RPS
      if (RobotBase.isReal()) {
        leftMetersPerSecond =
            driveMotors.get(MotorPosition.LEFT_FRONT).getSelectedSensorVelocity()
                * Constants.DriveTrain.kEncoderDistancePerPulseMeters
                * 10.0;
        rightMetersPerSecond =
            driveMotors.get(MotorPosition.RIGHT_FRONT).getSelectedSensorVelocity()
                * Constants.DriveTrain.kEncoderDistancePerPulseMeters
                * 10.0;
      } else {
        leftMetersPerSecond = m_drivetrainSimulator.getLeftVelocityMetersPerSecond();
        rightMetersPerSecond = m_drivetrainSimulator.getRightVelocityMetersPerSecond();
      }
    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public Pose2d getRobotPoseMeters() {
    if (RobotBase.isReal()) {
      return odometry.getPoseMeters();
    } else {
      return m_drivetrainSimulator.getPose();
    }
  }

  public DifferentialDriveKinematics getDriveTrainKinematics() {
    return kinematics;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  // TODO look into how odometry.resetPosition() should work. Apparently it takes a gyro angle?
  /** Resets the odometry and navX to a specified pose and heading */
  public void resetOdometry(Pose2d pose, Rotation2d rotation) {
    resetEncoderCounts();
    navX.reset();
    odometry.resetPosition(pose, rotation);
    if (RobotBase.isSimulation()) {
      // resetEncoderCounts();
      m_drivetrainSimulator.setPose(pose);
    }
  }

  private void updateSmartDashboard() {
    if (RobotBase.isReal()) {
      SmartDashboardTab.putNumber("DriveTrain", "Left Distance", getWheelDistanceMeters(MotorPosition.LEFT_FRONT));
      SmartDashboardTab.putNumber("DriveTrain", "Right Distance", getWheelDistanceMeters(MotorPosition.RIGHT_FRONT));
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "X Coordinate",
          Units.metersToFeet(getRobotPoseMeters().getTranslation().getX()));
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "Y Coordinate",
          Units.metersToFeet(getRobotPoseMeters().getTranslation().getY()));
      SmartDashboardTab.putNumber(
          "DriveTrain", "Angle", getRobotPoseMeters().getRotation().getDegrees());
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "Left Speed",
          Units.metersToFeet(getSpeedsMetersPerSecond().leftMetersPerSecond));
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "Right Speed",
          Units.metersToFeet(getSpeedsMetersPerSecond().rightMetersPerSecond));

      SmartDashboardTab.putNumber("Turret", "Robot Angle", getHeadingDegrees());
    } else {
      SmartDashboardTab.putNumber("DriveTrain", "Left Encoder", getEncoderCount(MotorPosition.LEFT_FRONT));
      SmartDashboardTab.putNumber("DriveTrain", "Right Encoder", getEncoderCount(MotorPosition.RIGHT_FRONT));
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "X Coordinate",
          Units.metersToFeet(getRobotPoseMeters().getTranslation().getX()));
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "Y Coordinate",
          Units.metersToFeet(getRobotPoseMeters().getTranslation().getY()));
      SmartDashboardTab.putNumber(
          "DriveTrain", "Angle", getRobotPoseMeters().getRotation().getDegrees());
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "Right Speed",
          Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "Right Speed",
          Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
      SmartDashboardTab.putNumber(
          "DriveTrain", "L Encoder Count", driveMotors.get(MotorPosition.LEFT_FRONT).getSelectedSensorPosition());
      SmartDashboardTab.putNumber(
          "DriveTrain", "R Encoder Count", driveMotors.get(MotorPosition.RIGHT_FRONT).getSelectedSensorPosition());
      SmartDashboardTab.putNumber(
          "DriveTrain", "L Encoder Rate", driveMotors.get(MotorPosition.LEFT_FRONT).getSelectedSensorVelocity());
      SmartDashboardTab.putNumber(
          "DriveTrain", "R Encoder Rate", driveMotors.get(MotorPosition.RIGHT_FRONT).getSelectedSensorVelocity());

      SmartDashboardTab.putNumber("Turret", "Robot Angle", getHeadingDegrees());
    }

    SmartDashboardTab.putNumber("DriveTrain", "Robot Angle", getHeadingDegrees());
    SmartDashboard.putBoolean("CTRE Feed Enabled", Unmanaged.getEnableState());
    SmartDashboardTab.putNumber("DriveTrain", "L Output", m_leftOutput);
    SmartDashboardTab.putNumber("DriveTrain", "R Output", m_rightOutput);
    SmartDashboardTab.putString("DriveTrain", "BrakeMode", neutralMode.toString());

    SmartDashboard.putData(
        "Set Neutral", new SetDriveTrainNeutralMode(this, DriveTrainNeutralMode.COAST));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboardTab.putNumber("DriveTrain", "WheelDistance", odometry.getPoseMeters().getX());
    odometry.update(
        Rotation2d.fromDegrees(getHeadingDegrees()),
        getWheelDistanceMeters(MotorPosition.LEFT_FRONT),
        getWheelDistanceMeters(MotorPosition.RIGHT_FRONT));
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.

    m_drivetrainSimulator.setInputs(
        m_leftOutput * RobotController.getBatteryVoltage(),
        m_rightOutput * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.040);

    // For CTRE devices, you must call this function periodically for simulation
    Unmanaged.feedEnable(40);
    driveMotors.get(MotorPosition.LEFT_FRONT)
        .getSimCollection()
        .setIntegratedSensorRawPosition( 
            (int)
                (m_drivetrainSimulator.getLeftPositionMeters()
                    / Constants.DriveTrain.kEncoderDistancePerPulseMeters));
    driveMotors.get(MotorPosition.LEFT_FRONT)
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_drivetrainSimulator.getLeftVelocityMetersPerSecond()
                    / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10.0)));
    driveMotors.get(MotorPosition.RIGHT_FRONT)
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_drivetrainSimulator.getRightPositionMeters()
                    / Constants.DriveTrain.kEncoderDistancePerPulseMeters));
    driveMotors.get(MotorPosition.RIGHT_FRONT)
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_drivetrainSimulator.getRightVelocityMetersPerSecond()
                    / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10.0)));
  }
}
