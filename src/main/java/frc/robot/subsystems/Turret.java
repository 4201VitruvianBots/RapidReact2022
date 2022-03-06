// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CONTROL_MODE;

public class Turret extends SubsystemBase {
  private final DriveTrain m_driveTrain;

  private final TalonFX turretMotor = new TalonFX(Constants.Turret.turretMotor);
  private final DigitalInput turretHomeSensor = new DigitalInput(Constants.Turret.turretHomeSensor);

  private double turretSetpointDegrees = 0; // angle

  private Constants.CONTROL_MODE controlMode = CONTROL_MODE.CLOSEDLOOP;
  private boolean usePoseEstimation = true;

  private double arbitraryFF;

  private boolean initialHome;
  private boolean turretHomeSensorLatch = false;

  /** Creates a new Turret. */
  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;

    turretMotor.configFactoryDefault();
    turretMotor.setInverted(false);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configVoltageCompSaturation(12.0);
    turretMotor.config_kP(0, Constants.Turret.kF);
    turretMotor.config_kP(0, Constants.Turret.kP);
    turretMotor.config_kI(0, Constants.Turret.kI);
    turretMotor.config_IntegralZone(0, Constants.Turret.kI_Zone);
    turretMotor.configMaxIntegralAccumulator(0, Constants.Turret.kMaxIAccum);
    turretMotor.config_kD(0, Constants.Turret.kD);
    turretMotor.configMotionCruiseVelocity(Constants.Turret.kCruiseVelocity);
    turretMotor.configMotionAcceleration(Constants.Turret.kMotionAcceleration);
    turretMotor.configAllowableClosedloopError(0, Constants.Turret.kErrorBand);
    //    turretMotor.setSensorPhase(true);
  }

  private void updateClosedLoopPosition() {
    double setpoint =
        Math.min(
            Math.max(turretSetpointDegrees, Constants.Turret.minAngle), Constants.Turret.maxAngle);

    SmartDashboardTab.putNumber("Turret", "Setpoint Test", setpoint);

    turretMotor.set(
        ControlMode.MotionMagic, degreesToEncoderUnits(setpoint) * Constants.Turret.gearRatio);
    //        DemandType.ArbitraryFeedForward,
    //        getArbitraryFF());
  }

  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0);
  }

  public Constants.CONTROL_MODE getControlMode() {
    return controlMode;
  }

  public void setControlMode(Constants.CONTROL_MODE mode) {
    controlMode = mode;
  }

  public boolean usePoseEstimation() {
    return usePoseEstimation;
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretAngleDegrees() {
    return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition())
        / Constants.Turret.gearRatio;
  }

  public Rotation2d getTurretRotation2d() {
    return new Rotation2d(Units.degreesToRadians(getTurretAngleDegrees()));
  }

  public Pose2d getPose() {
    return new Pose2d(
        m_driveTrain.getRobotPoseMeters().getTranslation(),
        m_driveTrain.getRobotPoseMeters().getRotation().plus(getTurretRotation2d()));
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretVelocity() {
    return encoderUnitsToDegrees(turretMotor.getSelectedSensorVelocity())
        / Constants.Turret.gearRatio;
  }

  /** double returns encoder units of turret into degrees */
  public double getEncoderAngle() {
    return turretMotor.getSelectedSensorPosition();
  }

  /** double returns encoder units of turret into degrees */
  public double getEncoderVelocity() {
    return turretMotor.getSelectedSensorVelocity();
  }

  /**
   * double
   *
   * @return turret angle - drivetrain angle
   */
  public double getFieldRelativeAngle() {
    return getTurretAngleDegrees() - m_driveTrain.getHeadingDegrees();
  }

  /**
   * double
   *
   * @return max angle
   */
  public double getMaxAngle() {
    return Constants.Turret.maxAngle;
  }

  /** @return minimum angle */
  public double getMinAngle() {
    return Constants.Turret.minAngle;
  }

  /**
   * boolean
   *
   * @return ! turrethomesensor.get
   */
  public boolean getTurretHome() {
    return !turretHomeSensor.get();
  }

  public double getTurretSetpointDegrees() {
    return encoderUnitsToDegrees(turretMotor.getClosedLoopTarget()) / Constants.Turret.gearRatio;
  }

  public boolean getInitialHome() {
    return initialHome;
  }

  /** @param output sets the percentoutput for the turretmotor */
  public void setPercentOutput(double output) {
    turretMotor.set(ControlMode.PercentOutput, output);
  }

  public void setAbsoluteSetpointDegrees(double turretSetpointDegrees) {
    this.turretSetpointDegrees = turretSetpointDegrees;
  }

  /** TurretSetpoint specifically sets the setpoint to turret angle */
  public void stopTurret() {
    turretSetpointDegrees = getTurretAngleDegrees();
  }

  public int degreesToEncoderUnits(double degrees) {
    return (int) (degrees * (Constants.Turret.encoderUnitsPerRotation / 360.0));
  }

  public double encoderUnitsToDegrees(double encoderUnits) {
    return encoderUnits * (360.0 / Constants.Turret.encoderUnitsPerRotation);
  }

  public boolean getTurretLatch() {
    return turretHomeSensorLatch;
  }

  // checks if the turret is pointing within the tolerance of the target
  public boolean onTarget() {
    return Math.abs(getTurretAngleDegrees() - getTurretSetpointDegrees())
        < Constants.Turret.kErrorBand;
  }

  public double getArbitraryFF() {
    return arbitraryFF;
  }

  public void setArbitraryFF(double arbitraryFF) {
    this.arbitraryFF = arbitraryFF;
  }

  private void updateShuffleboard() {
    if (RobotBase.isReal()) {
      SmartDashboardTab.putNumber("Turret", "Angle", getTurretAngleDegrees());
      SmartDashboardTab.putNumber("Turret", "Setpoint", getTurretSetpointDegrees());
      SmartDashboard.putNumber("Turret Angle", getTurretAngleDegrees());
    }
  }

  @Override
  public void periodic() {
    if (turretHomeSensor.get() && !turretHomeSensorLatch) {
      turretMotor.setSelectedSensorPosition(0);
      turretHomeSensorLatch = true;
    } else if (!turretHomeSensor.get() && turretHomeSensorLatch) {
      turretHomeSensorLatch = false;
    }

    updateClosedLoopPosition();
    updateShuffleboard();
  }
}
