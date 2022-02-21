// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Flywheel.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;
import java.io.File;

/** Creates a new Flywheel. */
public class Flywheel extends SubsystemBase {

  private final TalonFX[] flywheelMotors = {
    new TalonFX(Constants.Flywheel.flywheelMotorA), new TalonFX(Constants.Flywheel.flywheelMotorB)
  };

  private final Vision m_vision;
  private final Timer timeout = new Timer();
  public double rpmOutput;
  private double flywheelSetpointRPM;
  private boolean canShoot;
  private double idealRPM;
  private boolean timerStart = false;
  private Timer timer = new Timer();
  private double timestamp;

  private int testingSession = 0;

  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(Conversions.RpmToRadPerSec(rpmTolerance)), // Velocity error tolerance
          VecBuilder.fill(11.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  public Flywheel(Vision vision) {
    // Setup shooter motors (Falcons)
    for (TalonFX flywheelMotor : flywheelMotors) {
      flywheelMotor.configFactoryDefault();
      flywheelMotor.setNeutralMode(NeutralMode.Coast);
      flywheelMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
      flywheelMotor.configVoltageCompSaturation(11);
      flywheelMotor.enableVoltageCompensation(true);
    }
    flywheelMotors[0].setInverted(false);
    flywheelMotors[1].setInverted(true);
    flywheelMotors[1].follow(flywheelMotors[0], FollowerType.PercentOutput);

    m_vision = vision;
    m_controller.latencyCompensate(m_flywheelPlant, 0.02, 0.020);
  }
  /** @param output sets the controlmode percentoutput of outtakemotor0 */
  public void setPower(double output) {
    flywheelMotors[0].set(TalonFXControlMode.PercentOutput, output);
  }
  /** @param setpoint set to setpoint */
  public void setRPM(double flywheelSetpointRPM) {
    this.flywheelSetpointRPM = flywheelSetpointRPM;
  }

  /** @param setpoint set to setpoint */
  public double getSetpointRPM() {
    return flywheelSetpointRPM;
  }

  public void updateCanShoot() {
    if ((Math.abs(getSetpointRPM() - getRPM(0)) < getRPMTolerance() && !timerStart)) {
      timerStart = true;
      timer.reset();
      timer.start();
    } else if ((Math.abs(getSetpointRPM() - getRPM(0)) > getRPMTolerance()) && timerStart) {
      timerStart = false;
      timer.reset();
      timer.stop();
      canShoot = false;
    }

    if (timer.get() > 0.1) {
      canShoot = true;
    }
  }

  public boolean canShoot() {
    return canShoot;
  }

  /** flywheelSetpoint if setpoint else setPower to 0 */
  private void updateRPMSetpoint() {
    if (getSetpointRPM() > 0) {
      m_loop.setNextR(VecBuilder.fill(Conversions.RpmToRadPerSec(flywheelSetpointRPM)));

      m_loop.correct(VecBuilder.fill(Conversions.RpmToRadPerSec(getRPM(0))));

      m_loop.predict(0.020);

      double nextVoltage = m_loop.getU(0) + kFlywheelKs;

      if (timestamp <= 0.5 && timestamp > 0.4) {
        setPower(nextVoltage / 12.0);
      } else setPower(nextVoltage / 12.0);
    } else {
      setPower(0);
    }
  }

  /** set to test RPM */
  public void setTestRPM() {
    flywheelMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
  }

  public double getTestRPM() {
    return rpmOutput;
  }

  public double getRPMTolerance() {
    return rpmTolerance;
  }
  /**
   * boolean
   *
   * @param motorIndex
   * @return the absolute value of closed loop error < 100
   */
  public boolean encoderAtSetpoint(int motorIndex) {
    return (Math.abs(flywheelMotors[motorIndex].getClosedLoopError()) < 100.0);
  }

  /**
   * double
   *
   * @param motorIndex
   * @return falcon units to RPM with Flywheel velocity
   */
  public double getRPM(int motorIndex) {
    return flywheelMotors[motorIndex].getSelectedSensorVelocity()
        * (600.0 / encoderUnitsPerRotation)
        / gearRatio;
  }

  public double FalconUnitstoRPM(double SensorUnits) {
    return (SensorUnits / 2048.0) * 600.0 / gearRatio;
  }

  public double RPMtoFalconUnits(double RPM) {
    return (RPM / 600.0) * 2048.0 * gearRatio;
  }

  public void setIdealRPM() {
    flywheelSetpointRPM = idealRPM;
  }

  private void updateShuffleboard() {
    if (RobotBase.isReal()) {
      SmartDashboardTab.putNumber("Flywheel", "RPMPrimary", getRPM(0));
      SmartDashboardTab.putNumber("Flywheel", "RPMSecondary", getRPM(1));
      SmartDashboardTab.putNumber("Flywheel", "Power", flywheelMotors[0].getMotorOutputPercent());
      SmartDashboardTab.putNumber("Flywheel", "RPMSetpoint", flywheelSetpointRPM);
    }
  }

  /**
   * Returns the session name used for shooter logs.
   *
   * @return The session name
   */
  public String getTestingSessionName() {
    return "session" + testingSession;
  }

  public void updateTestingSession() {
    boolean success = false;
    try {
      while (!success) {
        File path = new File("/home/lvuser/frc/shooter_log/" + getTestingSessionName());
        success = path.mkdirs();
        testingSession++;
      }
      testingSession--;

    } catch (Exception e) {

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateRPMSetpoint();
    updateCanShoot();
    updateShuffleboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
