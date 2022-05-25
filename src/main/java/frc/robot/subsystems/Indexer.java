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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.PicoColorSensor;

public class Indexer extends SubsystemBase {
  private final double kI_Zone = 1;
  private final double maxVel = 1.1e4;
  private final double maxAccel = 1e6;
  private final double gearRatio = 1.0 / 27.0;
  // public TCA9548AcolorSensor colorSensor = new TCA9548AcolorSensor(I2C.Port.kMXP);
  private final PicoColorSensor colorSensor = new PicoColorSensor();

  private double voltageComp = 12.0;

  private DriverStation.Alliance frontColorType = DriverStation.Alliance.Invalid;
  private DriverStation.Alliance rearColorType = DriverStation.Alliance.Invalid;
  private Color frontColor = new Color(0, 0, 0);
  private Color rearColor = new Color(0, 0, 0);

  // Setup indexer motor controller (SparkMaxs)
  TalonFX ejectorMotor = new TalonFX(Constants.Indexer.ejectorMotor);
  TalonFX indexerMotor = new TalonFX(Constants.Indexer.indexerMotor); // RapidReact
  TalonFX kickerMotor = new TalonFX(Constants.Indexer.kickerMotor); // RapidReact

  // CANSparkMax indexerMotor =
  //     new CANSparkMax(Constants.Indexer.indexerMotor, MotorType.kBrushless); // Jango
  // VictorSPX kickerMotor = new VictorSPX(Constants.Indexer.kickerMotor); // Jango

  // Indexer sensors setup
  DigitalInput rearBeamBreak = new DigitalInput(Constants.Indexer.indexerRearSensor);
  DigitalInput frontBeamBreak = new DigitalInput(Constants.Indexer.indexerFrontSensor);

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
    indexerMotor.set(ControlMode.PercentOutput, output); // RapidReact

    // indexerMotor.set(output); // Jango
  }

   /**
   * Sets the power for the ejectpr motor
   *
   * @param output value for the power of the ejector motor
   */
  public void setEjectorPercentOutput(double output) {
    ejectorMotor.set(ControlMode.PercentOutput, output); // RapidReact
  }

  /**
   * front sensor tripped status
   *
   * @return boolean
   */
  public boolean getIndexerFrontSensorTripped() {
    return !frontBeamBreak.get();
  }

  /**
   * Gets the percent output of the indexer motor.
   *
   * @return the percent output of the indexer motor.
   */
  public double getIndexerOutput() {
    return indexerMotor.getMotorOutputPercent();
  }

  /**
   * Gets the percent output of the kicker motor.
   *
   * @return the percent output of the kicker motor.
   */
  public double getKickerOutput() {
    return kickerMotor.getMotorOutputPercent();
  }

  /**
   * rear sensor tripped status
   *
   * @return boolean
   */
  public boolean getIndexerRearSensorTripped() {
    return !rearBeamBreak.get();
  }

  /**
   * color value from mux channel
   *
   * @param channel
   * @return color
   */
  public Color getColor(int channel) {
    //   if (colorSensor.getMuxChannel() != channel) colorSensor.selectMuxChannel(channel);
    return channel == 1 ? colorSensor.getColor1() : colorSensor.getColor0();
  }

  /**
   * Returns int based on color detection
   *
   * @return Color of cargo
   */
  public DriverStation.Alliance getCargoColor(int channel) {
    Color color = getColor(channel);
    if (color.red > color.blue * 0.9 && color.red > color.green * 0.7) { //1.5, 0.7
      return DriverStation.Alliance.Red;
    } else if (color.blue > color.red * 0.7 && color.blue > color.green * 0.7) { //1.5
      return DriverStation.Alliance.Blue;
    } else return DriverStation.Alliance.Invalid;
  }

  /** Calls cargo color from tripped sensor */
  public void pollColorSensors() {
    if (getIndexerFrontSensorTripped()) {
      frontColorType = getCargoColor(0);
      frontColor = getColor(0);
    }
    if (getIndexerRearSensorTripped()) {
      rearColorType = getCargoColor(1);
      rearColor = getColor(1);
    }
  }

  /**
   * color from front sensor
   *
   * @return color
   */
  public DriverStation.Alliance getFrontColorType() {
    return frontColorType;
  }

  /**
   * color from rear sensor
   *
   * @return color
   */
  public DriverStation.Alliance getRearColorType() {
    return rearColorType;
  }

  /**
   * rgb from front sensor
   *
   * @return rgb value
   */
  public Color getFrontColor() {
    return frontColor;
  }

  /**
   * rgb from rear sensor
   *
   * @return rgb value
   */
  public Color getRearColor() {
    return rearColor;
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

    SmartDashboardTab.putBoolean("Indexer", "BeamBreakFront", getIndexerFrontSensorTripped());
    SmartDashboardTab.putBoolean("Indexer", "BeamBreakRear", getIndexerRearSensorTripped());

    SmartDashboardTab.putString("Indexer", "Front Color", getFrontColorType().toString());
    SmartDashboardTab.putNumber("Indexer", "Front Red", getFrontColor().red);
    SmartDashboardTab.putNumber("Indexer", "Front Green", getFrontColor().green);
    SmartDashboardTab.putNumber("Indexer", "Front Blue", getFrontColor().blue);
    SmartDashboardTab.putString("Indexer", "Rear Color", getRearColorType().toString());
    SmartDashboardTab.putNumber("Indexer", "Rear Red", getRearColor().red);
    SmartDashboardTab.putNumber("Indexer", "Rear Green", getRearColor().green);
    SmartDashboardTab.putNumber("Indexer", "Rear Blue", getRearColor().blue);
    SmartDashboardTab.putBoolean(
        "Indexer", "Front Color Connected", colorSensor.isSensor0Connected());
    SmartDashboardTab.putBoolean(
        "Indexer", "Rear Color Connected", colorSensor.isSensor1Connected());

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
            // * (600.0 / Constants.Flywheel.encoderUnitsPerRotation)
            // / Constants.Indexer.kickerGearRatio);
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
