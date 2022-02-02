// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Indexer.CARGO_COLOR;
import frc.robot.commands.intake.ReverseIntake;

public class Indexer extends SubsystemBase {
  private final double kI_Zone = 1;
  private final double maxVel = 1.1e4;
  private final double maxAccel = 1e6;
  private final double gearRatio = 1.0 / 27.0;
  public ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);

  // Setup indexer motor controller (SparkMaxs)
  TalonFX indexerMotor = new TalonFX(Constants.Indexer.indexerMotor);
  TalonFX kickerMotor = new TalonFX(Constants.Indexer.kickerMotor);

  // Indexer sensors setup
  DigitalInput rearBeamBreak = new DigitalInput(Constants.Indexer.indexerTopSensor);
  DigitalInput frontBeamBreak = new DigitalInput(Constants.Indexer.indexerBottomSensor);
  
  /** Creates a new Indexer. */
  public Indexer() {
    // Motor and PID controller setup
    indexerMotor.configFactoryDefault();
    indexerMotor.setInverted(true);

    indexerMotor.setNeutralMode(NeutralMode.Brake);

    kickerMotor.configFactoryDefault();
    kickerMotor.setInverted(true);

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
    indexerMotor.set(ControlMode.PercentOutput, output);
  }

  public boolean getIndexerFrontSensorTripped() {
    return !frontBeamBreak.get();
  }

  public boolean getIndexerRearSensorTripped() {
    return !rearBeamBreak.get();
  }

  public Color getColor() {
    return sensor.getColor();
  }
  
  /**
   * Returns int based on color detection
   * @return none = 0; red = 1; blue = 3
   */
  public int panelColor() { 
    if(getColor().red > getColor().blue * 3 && getColor().red > getColor().green * 1.33) {           
       return 1;
     } else if(getColor().blue < getColor().green * 1.15 && getColor().green < getColor().blue * 1.15 && getColor().blue > getColor().red * 2.5) {
      return 3;
    } else
       return 0;
  }

  /**
   * Checks cargo color based on panel color value
   * @return Cargo Color
   */
  public DriverStation.Alliance getCargoColor() {
    if(panelColor() == 1) {
      return DriverStation.Alliance.Red;
    } else if (panelColor() == 3) {
      return DriverStation.Alliance.Blue;
    } else {
      return DriverStation.Alliance.Invalid;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
