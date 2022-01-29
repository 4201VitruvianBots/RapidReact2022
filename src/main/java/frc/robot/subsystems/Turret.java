// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final int encoderUnitsPerRotation = 4096;
  private final DriveTrain m_driveTrain;
  private final Timer timeout = new Timer();
  private final CANCoder encoder = new CANCoder(Constants.Turret.turretEncoder);
  private final VictorSPX turretMotor = new VictorSPX(Constants.Turret.turretMotor);
  private final DigitalInput turretHomeSensor = new DigitalInput(Constants.Turret.turretHomeSensor);
  double minAngle = -90;
  double maxAngle = 90;
  private double m_angleOffset = -329.150;
  private double turretSetpointDegrees = 0; // angle
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
  // 1 = closed-loop control (using sensor feedback) and 0 = open-loop control (no sensor feedback)
  public enum TurretControlMode {
    OPENLOOP,
    CLOSEDLOOP
  };

  private TurretControlMode TurretControlMode;

  private boolean initialHome;
  private boolean turretHomeSensorLatch = false;

  private final LinearSystem<N2, N1, N1> m_turretPlant =
      LinearSystemId.identifyPositionSystem(
              kTurretKv,
              kTurretKa
      );

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_turretPlant,
          VecBuilder.fill(0.03, 0.3), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_turretPlant,
          VecBuilder.fill(Units.degreesToRadians(degreeTolerance),
                  Units.degreesToRadians(degreesPerSecondTolerance)), // Velocity error tolerance
          VecBuilder.fill(12.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(m_turretPlant, m_controller, m_observer, 12.0, 0.020);

  /** Creates a new Turret. */
  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;

    turretMotor.configFactoryDefault();
    turretMotor.setInverted(true);
    turretMotor.configVoltageCompSaturation(12.0);

    encoder.configFactoryDefault();
    encoder.configAllSettings(new CANCoderConfiguration());
    encoder.setPosition(0);

    m_loop.reset(VecBuilder.fill(
      Units.degreesToRadians(getEncoderAngle()),
      Units.degreesToRadians(getEncoderVelocity())));

    m_lastProfiledReference =
        new TrapezoidProfile.State(
            Units.degreesToRadians(getEncoderAngle()),
            Units.degreesToRadians(getEncoderVelocity()));

    m_controller.latencyCompensate(m_turretPlant, 0.02, 0.01);
  }

  private void updateDegreesSetpoint2() {
    double setpointDegrees = getTurretSetpointDegrees();
    setpointDegrees = Math.max(Math.min(setpointDegrees, 90), -90);

    double setpointRadians = Units.degreesToRadians(setpointDegrees) * canCodertoTurretGearRatio;
    TrapezoidProfile.State goal = new TrapezoidProfile.State(setpointRadians, 0.0);

    m_lastProfiledReference =
        (new TrapezoidProfile(Constants.Turret.TurretConstraints, goal, m_lastProfiledReference))
            .calculate(0.02);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    m_loop.correct(VecBuilder.fill(Units.degreesToRadians(getEncoderAngle())));

    m_loop.predict(0.020);

    double nextVoltage = m_loop.getU(0);

    setPercentOutput(nextVoltage / 12.0);
  }

  private void updateDegreesSetpoint() {
    /*double setpoint = getTurretSetpointDegrees();
    setpoint = Math.max(Math.min(setpoint, 90), -90);

    m_loop.setNextR(VecBuilder.fill(Units.degreesToRadians(setpoint * canCodertoTurretGearRatio)));

    m_loop.correct(
        VecBuilder.fill(Units.degreesToRadians(encoder.getPosition() * canCodertoTurretGearRatio)));

    m_loop.predict(0.020);

    double nextVoltage = m_loop.getU(0);

    setPercentOutput(nextVoltage / 12.0);
    */
  }

  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0);
    //  encoder.setPosition(0);
  }

  public TurretControlMode getControlMode() {
    return TurretControlMode;
  }

  public void setControlMode(TurretControlMode mode) {
    TurretControlMode = mode;
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretAngle() {
    return encoder.getPosition() / canCodertoTurretGearRatio;
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretVelocity() {
    return encoder.getVelocity() / canCodertoTurretGearRatio;
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
    return turretSetpointDegrees;
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

      SmartDashboard.putNumber("Turret Setpoint", turretSetpointDegrees);
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

    updateDegreesSetpoint2();
    updateShuffleboard();
  }
}
