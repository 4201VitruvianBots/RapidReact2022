// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.Constants.DriveTrain.MotorPosition;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import java.util.HashMap;
import java.util.Map;

/** A differential drivetrain with two falcon motors on each side */
public class DriveTrain extends SubsystemBase {

  // PID constants for the drive motors
  private final double kP = 1.2532;
  private final double kI = 0;
  private final double kD = 0;

  private final DifferentialDriveOdometry odometry;
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.DriveTrain.ksVolts,
          Constants.DriveTrain.kvVoltSecondsPerMeter,
          Constants.DriveTrain.kaVoltSecondsSquaredPerMeter);

  private final SimpleMotorFeedforward feedforwardCtre =
      new SimpleMotorFeedforward(
          Constants.DriveTrain.ksVolts / 12,
          Constants.DriveTrain.kvVoltSecondsPerMeter / 12,
          Constants.DriveTrain.kaVoltSecondsSquaredPerMeter / 12);

  PIDController leftPIDController = new PIDController(kP, kI, kD);
  PIDController rightPIDController = new PIDController(kP, kI, kD);

  /** Will run when enabled in teleop, unless null */
  private Command m_postAutoCommand = null;

  private final HashMap<MotorPosition, TalonFX> driveMotors =
      new HashMap<MotorPosition, TalonFX>(
          Map.of(
              MotorPosition.LEFT_FRONT, new TalonFX(Constants.DriveTrain.leftFrontDriveMotor),
              MotorPosition.LEFT_REAR, new TalonFX(Constants.DriveTrain.leftRearDriveMotor),
              MotorPosition.RIGHT_FRONT, new TalonFX(Constants.DriveTrain.rightFrontDriveMotor),
              MotorPosition.RIGHT_REAR, new TalonFX(Constants.DriveTrain.rightRearDriveMotor)));

  /** To hold on to the values to set the simulation motors */
  double m_leftOutput, m_rightOutput;

  private final Pigeon2 pigeon = new Pigeon2(Constants.DriveTrain.pigeonID, "rio");

  private DriveTrainNeutralMode neutralMode = DriveTrainNeutralMode.COAST;

  private DifferentialDrivetrainSim m_drivetrainSimulator;

  /** A differential drivetrain with two falcon motors on each side */
  public DriveTrain() {
    // Set up DriveTrain motors
    configureCtreMotors();

    // navX.reset();
    pigeon.setYaw(0);

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
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configForwardSoftLimitEnable(false);
      motor.configReverseSoftLimitEnable(false);
      motor.config_kP(0, 0.1);
      motor.configOpenloopRamp(0.25);
      motor.configClosedloopRamp(0.1);

      motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1));

      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    driveMotors.get(MotorPosition.LEFT_FRONT).setInverted(false);
    driveMotors.get(MotorPosition.LEFT_REAR).setInverted(false);
    driveMotors.get(MotorPosition.RIGHT_FRONT).setInverted(true);
    driveMotors.get(MotorPosition.RIGHT_REAR).setInverted(true);

    driveMotors.get(MotorPosition.LEFT_FRONT).setSensorPhase(false);
    driveMotors.get(MotorPosition.RIGHT_FRONT).setSensorPhase(false);

    driveMotors
        .get(MotorPosition.LEFT_REAR)
        .set(ControlMode.Follower, driveMotors.get(MotorPosition.LEFT_FRONT).getDeviceID());
    driveMotors
        .get(MotorPosition.RIGHT_REAR)
        .set(ControlMode.Follower, driveMotors.get(MotorPosition.RIGHT_FRONT).getDeviceID());
    driveMotors.get(MotorPosition.LEFT_REAR).setNeutralMode(NeutralMode.Brake);
    driveMotors.get(MotorPosition.RIGHT_REAR).setNeutralMode(NeutralMode.Brake);

    driveMotors.get(MotorPosition.LEFT_REAR).setStatusFramePeriod(1, 255);
    driveMotors.get(MotorPosition.LEFT_REAR).setStatusFramePeriod(2, 255);
    driveMotors.get(MotorPosition.RIGHT_REAR).setStatusFramePeriod(1, 255);
    driveMotors.get(MotorPosition.RIGHT_REAR).setStatusFramePeriod(2, 255);
  }

  /**
   * Gets the number of counts made by a specified encoder.
   *
   * @param position The position of the motor to get the encoder count of
   * @return The encoder count of the specified motor
   */
  public double getEncoderCount(MotorPosition position) {
    return driveMotors.get(position).getSelectedSensorPosition();
  }

  /**
   * Gets the heading of the robot.
   *
   * @return Clockwise negative heading of the robot in degrees
   */
  public double getHeadingDegrees() {
    return Math.IEEEremainder(pigeon.getYaw(), 360);
  }

  public void resetAngle() {
    pigeon.setYaw(0);
    pigeon.setAccumZAngle(0);
  }

  /**
   * Sets the angle adjustment of the NavX.
   *
   * @param angle the offset angle
   */
  public void setNavXOffsetDegrees(double angle) {
    pigeon.addYaw(angle);
  }

  /**
   * Gets the encoder position of a specified motor converted to wheel distance traveled.
   *
   * @param position The position of the motor to get the wheel distance of
   * @return The equivalent wheel distance traveled by the specified motor in meters
   */
  public double getWheelDistanceMeters(MotorPosition position) {
    return driveMotors.get(position).getSelectedSensorPosition()
        * Constants.DriveTrain.kEncoderDistancePerPulseMeters;
  }

  /**
   * Gets the input current of a specified motor.
   *
   * @param position The position of the motor to get the input current of
   * @return The input current of the specified motor in amps
   */
  public double getMotorInputCurrentAmps(MotorPosition position) {
    return driveMotors.get(position).getSupplyCurrent();
  }

  /**
   * Gets the current being drawn by the simulated drivetrain.
   *
   * @return The simulated current being drawn in amps
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /** Sets the encoder counts of all motors to 0 */
  public void resetEncoderCounts() {
    driveMotors.get(MotorPosition.LEFT_FRONT).setSelectedSensorPosition(0);
    driveMotors.get(MotorPosition.RIGHT_FRONT).setSelectedSensorPosition(0);
  }

  /**
   * Sets the drivetrain's motors using Arcade Drive: The forward/backward and rotational output can
   * be set independently.
   *
   * <p>Values are in percent output, but will be reduced proportionally to ensure no motor is set
   * to above 1.0 output
   *
   * @param throttle The forward/backward speed of the drivetrain (positive forward, negative
   *     backward)
   * @param turn The rotation of the drivetrain (positive clockwise, negative counterclockwise)
   */
  public void setMotorArcadeDrive(double throttle, double turn) {
    double leftOutput = throttle + turn;
    double rightOutput = throttle - turn;

    // Normalization
    double magnitude = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));
    if (magnitude > 1.0) {
      leftOutput /= magnitude;
      rightOutput /= magnitude;
    }

    // setMotorPercentOutput(leftOutput, rightOutput);
    setMotorTankDrive(leftOutput, rightOutput);
  }

  /**
   * Sets the drivetrain's motors using Tank Drive: The speeds of each sides are set individually.
   * Setting the left and right sides to the same value will make the drivetrain move straight,
   * setting the speeds to different values will cause the drivetrain to turn or curve.
   *
   * <p>Values are in percent output (positive forward, negative backward).
   *
   * @param leftOutput The output for the left side of the drivetrain
   * @param rightOutput The output for the right side of the drivetrain
   */
  public void setMotorTankDrive(double leftOutput, double rightOutput) {
    setMotorVelocityMetersPerSecond(
        leftOutput * Constants.DriveTrain.kMaxVelocityMetersPerSecond,
        rightOutput * Constants.DriveTrain.kMaxVelocityMetersPerSecond);
    //    setMotorPercentOutput(leftOutput, rightOutput);
  }

  /**
   * Calculates the percent output to send to the motors using the desired voltage and the current
   * battery voltage.
   *
   * <p>If either desired voltage is greater than the robot's current voltage, the amounts will be
   * reduced proportionally to ensure no motor is set above the available voltage.
   *
   * <p>Positive is forward, negative is backward.
   *
   * @param leftVoltage The voltage for the left side of the drivetrain
   * @param rightVoltage The voltage for the right side of the drivetrain
   */
  public void setVoltageOutput(double leftVoltage, double rightVoltage) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage)) > batteryVoltage) {
      leftVoltage *= batteryVoltage / 12.0;
      rightVoltage *= batteryVoltage / 12.0;
    }
    m_leftOutput = leftVoltage / batteryVoltage;
    m_rightOutput = rightVoltage / batteryVoltage;
    SmartDashboardTab.putNumber("DriveTrain", "Left Voltage", leftVoltage);
    SmartDashboardTab.putNumber("DriveTrain", "Right Voltage", rightVoltage);

    setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);
  }

  /**
   * Sets the percent output of the drivetrain.
   *
   * @param leftOutput The output for the left side of the drivetrain
   * @param rightOutput The output for the right side of the drivetrain
   */
  private void setMotorPercentOutput(double leftOutput, double rightOutput) {
    m_leftOutput = leftOutput;
    m_rightOutput = rightOutput;
    driveMotors.get(MotorPosition.LEFT_FRONT).set(ControlMode.PercentOutput, leftOutput);
    driveMotors.get(MotorPosition.RIGHT_FRONT).set(ControlMode.PercentOutput, rightOutput);
  }

  /**
   * Sets the velocity of the drivetrain, also taking into account feedforward.
   *
   * <p>Values are in meters per second (positive forward, negative backward).
   *
   * @param leftSpeed The velocity for the left side of the drivetrain
   * @param rightSpeed The velocity for the right side of the drivetrain
   */
  public void setMotorVelocityMetersPerSecond(double leftSpeed, double rightSpeed) {

    driveMotors
        .get(MotorPosition.LEFT_FRONT)
        .set(
            ControlMode.Velocity,
            leftSpeed / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10),
            DemandType.ArbitraryFeedForward,
            feedforwardCtre.calculate(leftSpeed));
    driveMotors
        .get(MotorPosition.RIGHT_FRONT)
        .set(
            ControlMode.Velocity,
            rightSpeed / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10),
            DemandType.ArbitraryFeedForward,
            feedforwardCtre.calculate(rightSpeed));
    m_leftOutput = leftSpeed / Constants.DriveTrain.kMaxVelocityMetersPerSecond;
    m_rightOutput = rightSpeed / Constants.DriveTrain.kMaxVelocityMetersPerSecond;
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

  /**
   * Gets the encoder position of a specified motor converted to wheel distance traveled.
   *
   * @param position The position of the motor to get the wheel distance of
   * @return The equivalent wheel distance traveled by the specified motor in meters
   */

  /**
   * Gets the speeds of both sides of the drivetrain converted to wheel speeds.
   *
   * @return A {@link DifferentialDriveWheelSpeeds} containing the speeds of each side in meters per
   *     second.
   */
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

  /**
   * Gets the feedforward.
   *
   * @return The {@link SimpleMotorFeedforward}
   */
  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  // TODO: The robot moves too quickly in teleop simulation, perhaps m_drivetrainSimulator
  // does not calculate the speeds correctly (possibly in DriveTrain.simulationPeriodic())
  public Pose2d getRobotPoseMeters() {
    if (RobotBase.isReal()) {
      return odometry.getPoseMeters();
    } else {
      return m_drivetrainSimulator.getPose();
    }
  }

  /**
   * Gets the drivetrain kinematics.
   *
   * @return The {@link DifferentialDriveKinematics}
   */
  public DifferentialDriveKinematics getDriveTrainKinematics() {
    return Constants.DriveTrain.kDriveKinematics;
  }

  /**
   * Gets the PID controller for the left side of the drivetrain.
   *
   * @return A {@link PIDController}
   */
  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  /**
   * Gets the PID controller for the right side of the drivetrain.
   *
   * @return A {@link PIDController}
   */
  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  // TODO look into how odometry.resetPosition() should work. Apparently it takes a gyro angle?
  /**
   * Resets the odometry and navX to a specified pose and heading.
   *
   * @param pose The new pose of the robot
   * @param rotation The current gyro angle of the robot
   */
  public void resetOdometry(Pose2d pose, Rotation2d rotation) {
    resetEncoderCounts();
    pigeon.setYaw(0);
    odometry.resetPosition(pose, rotation);
    if (RobotBase.isSimulation()) {
      // resetEncoderCounts();
      m_drivetrainSimulator.setPose(pose);
    }
  }

  /** Puts values on SmartDashboard. */
  private void updateSmartDashboard() {
    if (RobotBase.isReal()) {
      SmartDashboardTab.putNumber(
          "DriveTrain", "Left Distance", getWheelDistanceMeters(MotorPosition.LEFT_FRONT));
      SmartDashboardTab.putNumber(
          "DriveTrain", "Right Distance", getWheelDistanceMeters(MotorPosition.RIGHT_FRONT));
      SmartDashboardTab.putNumber(
          "DriveTrain", "X Coordinate", getRobotPoseMeters().getTranslation().getX());
      SmartDashboardTab.putNumber(
          "DriveTrain", "Y Coordinate", getRobotPoseMeters().getTranslation().getY());
      SmartDashboardTab.putNumber(
          "DriveTrain", "Angle", getRobotPoseMeters().getRotation().getDegrees());
      SmartDashboardTab.putNumber(
          "DriveTrain", "Left Speed", getSpeedsMetersPerSecond().leftMetersPerSecond);
      SmartDashboardTab.putNumber(
          "DriveTrain", "Right Speed", getSpeedsMetersPerSecond().rightMetersPerSecond);

      SmartDashboardTab.putNumber("Turret", "Robot Angle", getHeadingDegrees());
    } else {
      SmartDashboardTab.putNumber(
          "DriveTrain", "Left Encoder", getEncoderCount(MotorPosition.LEFT_FRONT));
      SmartDashboardTab.putNumber(
          "DriveTrain", "Right Encoder", getEncoderCount(MotorPosition.RIGHT_FRONT));
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
          "DriveTrain",
          "L Encoder Count",
          driveMotors.get(MotorPosition.LEFT_FRONT).getSelectedSensorPosition());
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "R Encoder Count",
          driveMotors.get(MotorPosition.RIGHT_FRONT).getSelectedSensorPosition());
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "L Encoder Rate",
          driveMotors.get(MotorPosition.LEFT_FRONT).getSelectedSensorVelocity());
      SmartDashboardTab.putNumber(
          "DriveTrain",
          "R Encoder Rate",
          driveMotors.get(MotorPosition.RIGHT_FRONT).getSelectedSensorVelocity());

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

  /**
   * Sets a command to run when enabled in teleop Does not run any command if {@code command} is
   * null
   *
   * @param command The command to run
   */
  public void setPostAutoCommand(Command command) {
    m_postAutoCommand = command;
  }

  /**
   * Gets the command to run when enabled in teleop
   *
   * @return The command to run
   */
  public Command getPostAutoCommand() {
    return m_postAutoCommand;
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
    driveMotors
        .get(MotorPosition.LEFT_FRONT)
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_drivetrainSimulator.getLeftPositionMeters()
                    / Constants.DriveTrain.kEncoderDistancePerPulseMeters));
    driveMotors
        .get(MotorPosition.LEFT_FRONT)
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_drivetrainSimulator.getLeftVelocityMetersPerSecond()
                    / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10.0)));
    driveMotors
        .get(MotorPosition.RIGHT_FRONT)
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_drivetrainSimulator.getRightPositionMeters()
                    / Constants.DriveTrain.kEncoderDistancePerPulseMeters));
    driveMotors
        .get(MotorPosition.RIGHT_FRONT)
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_drivetrainSimulator.getRightVelocityMetersPerSecond()
                    / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10.0)));
  }
}
