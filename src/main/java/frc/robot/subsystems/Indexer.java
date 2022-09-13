// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private double voltageComp = 12.0;

  // Setup indexer motor controller
  TalonFX indexerMotor = new TalonFX(Constants.Indexer.indexerMotor);
  TalonFX kickerMotor = new TalonFX(Constants.Indexer.kickerMotor);

  private double kickerSetpoint;

  private final LinearSystem<N1, N1, N1> m_KickerPlant =
      LinearSystemId.identifyVelocitySystem(
          Constants.Indexer.kKickerKv, Constants.Indexer.kKickerKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_KickerPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_KickerPlant,
          VecBuilder.fill(Constants.Indexer.radiansPerSecondTolerance), // Velocity error tolerance
          VecBuilder.fill(12.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_KickerPlant, m_controller, m_observer, 12.0, 0.020);

  /** Creates a new Indexer. */
  public Indexer() {
    // Motor and PID controller setup
    indexerMotor.configFactoryDefault();
    indexerMotor.setInverted(false);

    indexerMotor.setStatusFramePeriod(1, 100);
    indexerMotor.setStatusFramePeriod(2, 100);
    kickerMotor.configFactoryDefault();
    kickerMotor.setInverted(false);
    kickerMotor.configVoltageCompSaturation(voltageComp);
    kickerMotor.enableVoltageCompensation(true);

    kickerMotor.setNeutralMode(NeutralMode.Brake);

    kickerMotor.setStatusFramePeriod(1, 100);
    kickerMotor.setStatusFramePeriod(2, 100);

    // SmartDashboard.putData("indexer Subsystem", this);

    m_controller.latencyCompensate(m_KickerPlant, 0.02, 0.01);
  }

  /**
   * sets the power for the kicker motor
   *
   * @param output value for the power of the kicker motor
   */
  public void setKickerPercentOutput(double output) {
    kickerSetpoint = output * Constants.Indexer.falconMaxSpeedRadPerSecond;
  }

  public void setKickerPower(double output) {
    kickerMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Sets the power for the indexer motor
   *
   * @param output value for the power of the indexer motor
   */
  public void setIndexerPercentOutput(double output) {
    indexerMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Gets the percent output of the kicker motor.
   *
   * @return the percent output of the kicker motor.
   */
  public double getKickerOutput() {
    return kickerMotor.getMotorOutputPercent();
  }

  private void updateSetpoint() {
    if (Math.abs(kickerSetpoint) > 0) {
      m_loop.setNextR(VecBuilder.fill(kickerSetpoint));
      m_loop.correct(
          VecBuilder.fill(
              kickerMotor.getSelectedSensorVelocity()
                  * (10.0 * 2.0 * Math.PI / Constants.Flywheel.encoderUnitsPerRotation)));
      m_loop.predict(0.02);
      setKickerPower((m_loop.getU(0) + Constants.Indexer.kKickerKs) / voltageComp);
    } else {
      setKickerPower(0);
    }
  }

  @Override
  public void periodic() {
    updateSetpoint();
    SmartDashboardTab.putNumber(
        "Indexer",
        "Indexer Speed",
        indexerMotor.getSelectedSensorVelocity()
            * (10.0
                * 2.0
                * Math.PI
                / (Constants.Flywheel.encoderUnitsPerRotation
                    * Constants.Indexer.falconMaxSpeedRadPerSecond)));
    SmartDashboardTab.putNumber(
        "Indexer",
        "Kicker Speed",
        kickerMotor.getSelectedSensorVelocity()
            * (10.0
                * 2.0
                * Math.PI
                / (Constants.Flywheel.encoderUnitsPerRotation
                    * Constants.Indexer.falconMaxSpeedRadPerSecond)));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
