// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final DriveTrain m_driveTrain;

  private final CANCoder encoder = new CANCoder(Constants.Turret.turretEncoder);
  private final VictorSPX turretMotor = new VictorSPX(Constants.Turret.turretMotor);
  private final DigitalInput turretHomeSensor = new DigitalInput(Constants.Turret.turretHomeSensor);

  private double turretSetpointDegrees = 0; // angle

  private Constants.CONTROL_MODE controlMode;

  private boolean initialHome;
  private boolean turretHomeSensorLatch = false;

  /** Creates a new Turret. */
  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;

    turretMotor.configFactoryDefault();
    turretMotor.setInverted(true);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configVoltageCompSaturation(12.0);
    turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
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

    encoder.configFactoryDefault();
    encoder.configAllSettings(new CANCoderConfiguration());
    encoder.setPosition(0);
  }

  private void updateClosedLoopPosition() {
    double setpoint = Math.min(Math.max(turretSetpointDegrees, minAngle), maxAngle);

    turretMotor.set(
        ControlMode.MotionMagic, degreesToEncoderUnits(setpoint) * canCoderToTurretGearRatio);
  }

  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0);
    encoder.setPosition(0);
  }

  public Constants.CONTROL_MODE getControlMode() {
    return controlMode;
  }

  public void setControlMode(Constants.CONTROL_MODE mode) {
    controlMode = mode;
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretAngle() {
    return encoder.getPosition() / canCoderToTurretGearRatio;
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretVelocity() {
    return encoder.getVelocity() / canCoderToTurretGearRatio;
  }

  /** double returns encoder units of turret into degrees */
  public double getEncoderAngle() {
    return encoder.getPosition();
  }

  /** double returns encoder units of turret into degrees */
  public double getEncoderVelocity() {
    return encoder.getVelocity();
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
    return encoderUnitsToDegrees(turretMotor.getClosedLoopTarget()) / canCoderToTurretGearRatio;
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
      SmartDashboard.putNumber("Turret Angle", getTurretAngle());

      SmartDashboard.putNumber("Turret Setpoint", getTurretSetpointDegrees());
      SmartDashboard.putNumber("Turret Error", turretMotor.getClosedLoopError());
      SmartDashboard.putNumber("Turret Victor Angle", turretMotor.getSelectedSensorPosition());
    }
  }

  @Override
  public void periodic() {
    if (turretHomeSensor.get() && !turretHomeSensorLatch) {
      encoder.setPosition(0);
      turretHomeSensorLatch = true;
    } else if (!turretHomeSensor.get() && turretHomeSensorLatch) {
      turretHomeSensorLatch = false;
    }

    // updateClosedLoopPosition();
    updateShuffleboard();
  }
}
