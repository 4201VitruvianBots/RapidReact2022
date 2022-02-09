// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * climber will only be making for a mid-climb as of 1/11/22 in the future I hope that the climber
 * will use some more advanced capability to get a traversal run climb in the future.
 */
public class Climber extends SubsystemBase {
  private DigitalInput climberLowerLimitOverride =
      new DigitalInput(Constants.Climber.climberLowerLimitOverrideID);

  private DigitalInput climberUpperLimitOverride =
      new DigitalInput(Constants.Climber.climberUpperLimitOverrideID);

  private boolean Overridelatched = false;

  private final TalonFX[] climbMotors = {
    new TalonFX(Constants.Climber.climbMotorA),
    new TalonFX(Constants.Climber.climbMotorB),
    new TalonFX(Constants.Climber.highClimbMotor)
  };
  private boolean climbState;

  private final double upperLimit = 205_000.0;
  private final double lowerLimit = 0.0;

  /** Creates a new Climber. */
  public Climber() {
    // Set up climber motor
    for (int i = 0; i < climbMotors.length; i++) {
      climbMotors[i].configFactoryDefault();
      climbMotors[i].setSelectedSensorPosition(0);
      climbMotors[i].setNeutralMode(NeutralMode.Brake);
      climbMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    climbMotors[1].set(TalonFXControlMode.Follower, climbMotors[0].getDeviceID());
  }

  public boolean getClimbState() {
    return climbState;
  }

  public void setClimbState(boolean climbState) {
    this.climbState = climbState;
  }

  /**
   * sets the climber motor's power with a percent (0.0 - 1.0)
   *
   * @param value output value
   */
  public void setClimberPercentOutput(double value) {
    if ((getClimberPosition() > lowerLimit || value > 0)
        && (getClimberPosition() < upperLimit || value < 0))
      climbMotors[0].set(ControlMode.PercentOutput, value);
    else climbMotors[0].set(ControlMode.PercentOutput, 0);
  }

  public void setHighClimberPercentOutput(double value) {
    climbMotors[2].set(ControlMode.PercentOutput, value);
  }

  /**
   * get the climber position
   *
   * @return the climber position (in raw sensor units)
   */
  public double getClimberPosition() {
    return climbMotors[0].getSelectedSensorPosition();
  }

  /**
   * get the high climber position
   *
   * @return the climber position (in raw sensor units)
   */
  public double getHighClimberPosition() {
    return climbMotors[2].getSelectedSensorPosition();
  }

  private void updateClimberLimits() {
    if (!Overridelatched) {
      if (!climberLowerLimitOverride.get()) {
        climbMotors[0].setSelectedSensorPosition(lowerLimit);
        Overridelatched = true;
      } else if (!climberUpperLimitOverride.get()) {
        climbMotors[0].setSelectedSensorPosition(upperLimit);
        Overridelatched = true;
      }
    } else if (Overridelatched
        && climberLowerLimitOverride.get()
        && climberUpperLimitOverride.get()) {
      Overridelatched = false;
    }
  }

  private void updateSmartDashboard() {
    SmartDashboardTab.putBoolean("Climber", "Climb Mode", getClimbState());
    SmartDashboardTab.putNumber("Climber", "Climb Output", climbMotors[0].getMotorOutputPercent());
    SmartDashboardTab.putNumber("Climber", "Climb Position", getClimberPosition());
    SmartDashboardTab.putBoolean(
        "Climber", "ClimberLowerOverride", climberLowerLimitOverride.get());
    SmartDashboardTab.putBoolean(
        "Climber", "ClimberUpperOverride", climberUpperLimitOverride.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();

    if ((getClimberPosition() <= lowerLimit && climbMotors[0].getMotorOutputPercent() < 0)
        || (getClimberPosition() >= upperLimit && climbMotors[0].getMotorOutputPercent() > 0))
      climbMotors[0].set(ControlMode.PercentOutput, 0);

    updateClimberLimits();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
