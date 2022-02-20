// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.vitruvianlib.utils.TCA9548AcolorSensor;

public class Indexer extends SubsystemBase {
  private final double kI_Zone = 1;
  private final double maxVel = 1.1e4;
  private final double maxAccel = 1e6;
  private final double gearRatio = 1.0 / 27.0;
  public TCA9548AcolorSensor colorSensor = new TCA9548AcolorSensor(I2C.Port.kMXP);

  private DriverStation.Alliance frontColorType = DriverStation.Alliance.Invalid;
  private DriverStation.Alliance rearColorType = DriverStation.Alliance.Invalid;
  private Color frontColor = new Color(0, 0, 0);
  private Color rearColor = new Color(0, 0, 0);

  // Setup indexer motor controller (SparkMaxs)

  TalonFX indexerMotor = new TalonFX(Constants.Indexer.indexerMotor); // RapidReact
  TalonFX kickerMotor = new TalonFX(Constants.Indexer.kickerMotor); // RapidReact

  // CANSparkMax indexerMotor =
  //     new CANSparkMax(Constants.Indexer.indexerMotor, MotorType.kBrushless); // Jango
  // VictorSPX kickerMotor = new VictorSPX(Constants.Indexer.kickerMotor); // Jango

  // Indexer sensors setup
  DigitalInput rearBeamBreak = new DigitalInput(Constants.Indexer.indexerRearSensor);
  DigitalInput frontBeamBreak = new DigitalInput(Constants.Indexer.indexerFrontSensor);

  /** Creates a new Indexer. */
  public Indexer() {
    // Motor and PID controller setup
    indexerMotor.configFactoryDefault();
    indexerMotor.setInverted(false);

    indexerMotor.setStatusFramePeriod(1, 100);
    indexerMotor.setStatusFramePeriod(2, 100);
    kickerMotor.configFactoryDefault();
    kickerMotor.setInverted(false);

    kickerMotor.setNeutralMode(NeutralMode.Brake);

    kickerMotor.setStatusFramePeriod(1, 100);
    kickerMotor.setStatusFramePeriod(2, 100);

    SmartDashboard.putData("indexer Subsystem", this);
  }

  /**
   * sets the power for the kicker motor
   *
   * @param output value for the power of the kicker motor
   */
  public void setKickerPercentOutput(double output) {
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
  
  /*
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
    if (colorSensor.getMuxChannel() != channel) colorSensor.selectMuxChannel(channel);
    return colorSensor.getColorSensor().getColor();
  }

  /**
   * Returns int based on color detection
   *
   * @return Color of cargo
   */
  public DriverStation.Alliance getCargoColor(int channel) {
    Color color = getColor(channel);
    if (color.red > color.blue * 1.5 && color.red > color.green * 0.7) {
      return DriverStation.Alliance.Red;
    } else if (color.blue > color.red * 1.5 && color.blue > color.green * 0.7) {
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
      rearColorType = getCargoColor(2);
      rearColor = getColor(2);
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

  @Override
  public void periodic() {
    SmartDashboardTab.putBoolean("Indexer", "BeamBreakFront", getIndexerFrontSensorTripped());
    SmartDashboardTab.putBoolean("Indexer", "BeamBreakRear", getIndexerRearSensorTripped());

    SmartDashboardTab.putString("Indexer", "Rear Color", getFrontColorType().toString());
    SmartDashboardTab.putNumber("Indexer", "Rear Red", getFrontColor().red);
    SmartDashboardTab.putNumber("Indexer", "Rear Green", getFrontColor().green);
    SmartDashboardTab.putNumber("Indexer", "Rear Blue", getFrontColor().blue);
    SmartDashboardTab.putString("Indexer", "Front Color", getRearColorType().toString());
    SmartDashboardTab.putNumber("Indexer", "Front Red", getRearColor().red);
    SmartDashboardTab.putNumber("Indexer", "Front Green", getRearColor().green);
    SmartDashboardTab.putNumber("Indexer", "Front Blue", getRearColor().blue);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
