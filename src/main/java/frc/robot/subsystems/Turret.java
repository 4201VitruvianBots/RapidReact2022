// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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

  private boolean initialHome;
  private boolean turretHomeSensorLatch = false;

  /** Creates a new Turret. */
  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;

    turretMotor.configFactoryDefault();
    turretMotor.setInverted(true);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configVoltageCompSaturation(12.0);
    turretMotor.config_kP(0, kF);
    turretMotor.config_kP(0, kP);
    turretMotor.config_kI(0, kI);
    turretMotor.config_IntegralZone(0, kI_Zone);
    turretMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
    turretMotor.config_kD(0, kD);
    turretMotor.configMotionCruiseVelocity(kCruiseVelocity);
    turretMotor.configMotionAcceleration(kMotionAcceleration);
    turretMotor.configAllowableClosedloopError(0, kErrorBand);
    turretMotor.setSensorPhase(true);
  }

  private void updateClosedLoopPosition() {
    double setpoint = Math.min(Math.max(turretSetpointDegrees, minAngle), maxAngle);

    SmartDashboard.putNumber("Turret Setpoint Test", setpoint);

    turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(setpoint) * gearRatio);
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

  /** double returns encoder units of turret into degrees */
  public double getTurretAngle() {
    return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition()) / gearRatio;
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretVelocity() {
    return encoderUnitsToDegrees(turretMotor.getSelectedSensorVelocity()) / gearRatio;
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
    return getTurretAngle() - m_driveTrain.getHeadingDegrees();
  }

  /**
   * double
   *
   * @return max angle
   */
  public double getMaxAngle() {
    return maxAngle;
  }

  /** @return minimum angle */
  public double getMinAngle() {
    return minAngle;
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
    return encoderUnitsToDegrees(turretMotor.getClosedLoopTarget()) / gearRatio;
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
    turretSetpointDegrees = getTurretAngle();
  }

  public int degreesToEncoderUnits(double degrees) {
    return (int) (degrees * (encoderUnitsPerRotation / 360.0));
  }

  public double encoderUnitsToDegrees(double encoderUnits) {
    return encoderUnits * (360.0 / encoderUnitsPerRotation);
  }

  public boolean getTurretLatch() {
    return turretHomeSensorLatch;
  }

  // checks if the turret is pointing within the tolerance of the target
  public boolean onTarget() {
    return Math.abs(getTurretAngle() - getTurretSetpointDegrees()) < kErrorBand;
  }

  private void updateShuffleboard() {
    if (RobotBase.isReal()) {
      SmartDashboardTab.putNumber("Turret", "Angle", getTurretAngle());
      SmartDashboardTab.putNumber("Turret", "Setpoint", getTurretSetpointDegrees());
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
