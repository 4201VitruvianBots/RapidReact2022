// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;

public class Turret extends SubsystemBase {
  private final int encoderUnitsPerRotation = 4096;
  private final DriveTrain m_driveTrain;
  private final Timer timeout = new Timer();
  private final CANCoder encoder = new CANCoder(Constants.Turret.turretEncoder);
  private final VictorSPX turretMotor = new VictorSPX(Constants.Turret.turretMotor);
  private final DigitalInput turretHomeSensor = new DigitalInput(Constants.Turret.turretHomeSensor);
  double minAngle = -90;
  double maxAngle = 90;
  private double turretSetpointDegrees = 0; // angle
  // 1 = closed-loop control (using sensor feedback) and 0 = open-loop control (no sensor feedback)
  public enum TurretControlMode {
    OPENLOOP,
    CLOSEDLOOP
  };

  private TurretControlMode TurretControlMode;

  private boolean initialHome;
  private boolean turretHomeSensorLatch = false;

  private final LinearSystem<N1, N1, N1> m_turretPlant =
      LinearSystemId.identifyVelocitySystem(kTurretKv, kTurretKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_turretPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_turretPlant,
          VecBuilder.fill(
              Conversions.DegreestoRadPerSec(degreeTolerance)), // Velocity error tolerance
          VecBuilder.fill(12.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_turretPlant, m_controller, m_observer, 12.0, 0.020);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.Turret.kTurretKs, Constants.Turret.kTurretKv);
  /** Creates a new Turret. */
  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    encoder.configFactoryDefault();
    encoder.setPositionToAbsolute();
    encoder.configSensorDirection(true);
  }

  private void updateDegreesSetpoint() {
    double setpoint = getTurretSetpointDegrees();
    setpoint = Math.max(Math.min(setpoint, 90), -90);

    m_loop.setNextR(VecBuilder.fill(Units.degreesToRadians(setpoint * canCodertoTurretGearRatio)));

    m_loop.correct(VecBuilder.fill(Units.degreesToRadians(getTurretAngle() * canCodertoTurretGearRatio)));

    m_loop.predict(0.020);

    double nextVoltage = m_loop.getU(0);

    setPercentOutput(nextVoltage / 12.0 + Constants.Turret.kTurretKs / 12.0);
  }

  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0);
    encoder.setPosition(0);
  }

  public TurretControlMode getControlMode() {
    return TurretControlMode;
  }

  public void setControlMode(TurretControlMode mode) {
    TurretControlMode = mode;
  }

  /** double returns encoder units of turret into degrees */
  public double getTurretAngle() {
    return encoderUnitsToDegrees(encoder.getPosition());
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

  public void setRobotCentricSetpoint(double turretSetpointDegrees) {
    this.turretSetpointDegrees = turretSetpointDegrees;
  }

  /** TurretSetpoint specifically sets the setpoint to turret angle */
  public void stopTurret() {
    turretSetpointDegrees = getTurretAngle();
  }

  public int degreestoEncoderunits(double degrees) {
    return (int) (degrees * (1.0 / canCodertoTurretGearRatio) * (encoderUnitsPerRotation / 360.0));
  }

  public double encoderUnitsToDegrees(double encoderUnits) {
    return encoderUnits / canCodertoTurretGearRatio;
  }

  public boolean getTurretLatch() {
    return turretHomeSensorLatch;
  }

  // checks if the turret is pointing within the tolerance of the target
  public boolean onTarget() {
    return Math.abs(turretMotor.getClosedLoopError()) < kErrorBand;
  }

  private void updateShuffleboard() {
    if (RobotBase.isReal()) {
      SmartDashboard.putNumber("Angle", getTurretAngle());

      SmartDashboard.putNumber("Setpoint", turretSetpointDegrees);
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
    updateDegreesSetpoint();
    updateShuffleboard();
  }
}
