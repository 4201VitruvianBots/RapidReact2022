// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final double kI_Zone = 1;
  private final double maxVel = 1.1e4;
  private final double maxAccel = 1e6;
  private final double gearRatio = 1.0 / 27.0;

  // Setup indexer motor controller (SparkMaxs)
  TalonFX master = new TalonFX(Constants.Indexer.indexerMotor);
  double encoder = master.getSelectedSensorVelocity();
  TalonFX kicker = new TalonFX(Constants.Indexer.kickerMotor);

  // PID terms/other constants
  private double kF = 0.0001;
  private double kP = 0.000001;
  private double kI = 80;
  private double kD = 0.0001;
  private int controlMode = 1;

  // Indexer sensors setup
  DigitalInput intakeSensor = new DigitalInput(Constants.Intake.intakeSensor);
  DigitalInput indexerTopSensor = new DigitalInput(Constants.Indexer.indexerTopSensor);
  DigitalInput indexerBottomSensor = new DigitalInput(Constants.Indexer.indexerBottomSensor);
  /** Creates a new Indexer. */
  public Indexer() {
    // Motor and PID controller setup
    master.configFactoryDefault();
    master.setInverted(true);

    master.setNeutralMode(NeutralMode.Coast);

    // TODO: What is a Slot ID
    master.config_kF(0, kF);
    master.config_kP(0, kP);
    master.config_kI(0, kI);
    master.config_kD(0, kD);
    // master.setSmartMotionMaxVelocity(maxVel, 0); // Formerly 1.1e4  TODO: What is a
    // SmartMotionVelocity
    // master.setSmartMotionMaxAccel(maxAccel, 0); // Formerly 1e6   TODO: What is a
    // SmartMotionVelocity
    // master.setSmartMotionAllowedClosedLoopError(1, 0);    TODO: What is a SmartMotionVelocity

    kicker.configFactoryDefault();
    kicker.setInverted(true);
  }

  /** Sets control mode to either 1 or 0 */
  public void toggleControlMode() {
    if (controlMode == 0) controlMode = 1;
    else controlMode = 0;
  }

  /** @return Returns the control mode value */
  public int getControlMode() {
    return controlMode;
  }

  /** @return Intake sensor activation status */
  public boolean getIntakeSensor() {
    return (!intakeSensor.get());
  }

  /**
   * sets the power for the kicker motor
   *
   * @param output value for the power of the kicker motor
   */
  public void setKickerOutput(double output) {
    kicker.set(ControlMode.PercentOutput, output);
  }

  /**
   * Sets the power for the indexer motor
   *
   * @param output value for the power of the indexer motor
   */
  public void setIndexerOutput(double output) {
    master.set(ControlMode.PercentOutput, output);
  }

  /**
   * Senses if there is a ball in the indexer don't know if we need it
   *
   * @return if there is a ball in the indexer
   */
  public boolean newBall() {
    return false;
  }

  public boolean getIndexerBottomSensor() {
    return !indexerBottomSensor.get();
  }

  public boolean getIndexerTopSensor() {
    return !indexerTopSensor.get();
  }

  /**
   * Sets the RPM of the indexer
   *
   * @param rpm value for the rpm
   */
  public void setRPM(double rpm) {
    double setpoint = rpm / gearRatio;
    SmartDashboard.putNumber("Indexer Setpoint", setpoint);
    // master.setReference(setpoint, TalonFX.ControlType.kSmartVelocity);
    master.set(
        ControlMode.PercentOutput,
        setpoint); // TODO: What is the TalonFX equivilent of SmartVelocity
  }

  /** updates the SmartDashboard with Indexer values */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("kF", kF);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
  }

  /** updates PID values onto the controllers and smartdashboard */
  private void updatePIDValues() {
    kF = SmartDashboard.getNumber("kF", 0);
    kP = SmartDashboard.getNumber("kP", 0);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0);
    master.config_kF(0, kF);
    master.config_kP(0, kP);
    master.config_kI(0, kI);
    master.config_kD(0, kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    updatePIDValues();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
