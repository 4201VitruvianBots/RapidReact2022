// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * climber will only be making for a mid-climb as of 1/11/22 in the future I hope that the climber
 * will use some more advanced capability to get a traversal run climb in the future.
 */
public class Climber extends SubsystemBase {
  public final TalonFX[] elevatorClimbMotors = {
    new TalonFX(Constants.Climber.climbMotorA), new TalonFX(Constants.Climber.climbMotorB)
  };

  private DigitalInput climberLowerLimitOverride =
      new DigitalInput(Constants.Climber.climberLowerLimitOverrideID);

  // private DigitalInput climberUpperLimitOverride =
  //     new DigitalInput(Constants.Climber.climberUpperLimitOverrideID);

  private boolean Overridelatched = false;
  private double climberPosition = 0;

  DoubleSolenoid highClimbPiston =
      new DoubleSolenoid(
          Constants.Pneumatics.pcmOne,
          Constants.Pneumatics.pcmType,
          Constants.Pneumatics.climbPistonForward,
          Constants.Pneumatics.climbPistonReverse);

  private final double kF = 0;
  private final double kP = 0.2;

  private boolean elevatorClimbState;

  private double holdPosition;

  /** Creates a new Climber. */
  public Climber() {
    // Set up climber motor
    for (int i = 0; i < elevatorClimbMotors.length; i++) {
      elevatorClimbMotors[i].configFactoryDefault();
      elevatorClimbMotors[i].setSelectedSensorPosition(0);
      elevatorClimbMotors[i].setNeutralMode(NeutralMode.Brake);
      elevatorClimbMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      elevatorClimbMotors[i].configSupplyCurrentLimit(
          new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1));
    }
    elevatorClimbMotors[1].set(TalonFXControlMode.Follower, elevatorClimbMotors[0].getDeviceID());

    elevatorClimbMotors[0].setInverted(false);
    elevatorClimbMotors[1].setInverted(false);
    elevatorClimbMotors[0].config_kF(0, kF);
    elevatorClimbMotors[0].config_kP(0, kP);

    elevatorClimbMotors[0].setStatusFramePeriod(1, 100);
    elevatorClimbMotors[0].setStatusFramePeriod(2, 100);

    elevatorClimbMotors[1].setStatusFramePeriod(1, 255);
    elevatorClimbMotors[1].setStatusFramePeriod(1, 255);
  }

  public boolean getElevatorClimbState() {
    return elevatorClimbState;
  }

  public void setElevatorClimbState(boolean climbState) {
    this.elevatorClimbState = climbState;
  }

  /**
   * sets the climber motor's power with a percent (0.0 - 1.0)
   *
   * @param value output value
   */
  public void setElevatorClimberPercentOutput(double value) {
    climberPosition = getElevatorClimbPosition();
    if (value > 0) {
      if (climberPosition < Constants.Climber.climberEncoderUpperLimit) {
        if (Math.abs(climberPosition - Constants.Climber.climberEncoderUpperLimit)
            < Constants.Climber.climberEncoderSlowdown) {
          elevatorClimbMotors[0].set(
              ControlMode.PercentOutput,
              Math.min(value, Constants.Climber.maxSpeedLimitsPercent)
                  * Math.abs(climberPosition - Constants.Climber.climberEncoderUpperLimit)
                  / Constants.Climber.climberEncoderSlowdown);
        } else {
          elevatorClimbMotors[0].set(ControlMode.PercentOutput, value);
        }
      } else elevatorClimbMotors[0].set(ControlMode.PercentOutput, 0);
    } else {
      if (!climberLowerLimitOverride.get()) {
        elevatorClimbMotors[0].set(ControlMode.PercentOutput, 0);
      } else {
        elevatorClimbMotors[0].set(ControlMode.PercentOutput, value);
      }
    }
  }

  public void setHighClimbPiston(DoubleSolenoid.Value kValue) {
    highClimbPiston.set(kValue);
  }

  public void holdClimber() {
    elevatorClimbMotors[0].set(ControlMode.Position, holdPosition);
  }

  public void setHoldPosition(double position) {
    holdPosition = position;
  }

  /**
   * get the climber position
   *
   * @return the climber position (in raw sensor units)
   */
  public double getElevatorClimbPosition() {
    return elevatorClimbMotors[0].getSelectedSensorPosition();
  }

  /**
   * get the high climber position
   *
   * @return the climber position (in raw sensor units)
   */
  public DoubleSolenoid.Value getHighClimbPistonPosition() {
    return highClimbPiston.get();
  }

  private void updateClimberLimits() {
    // if (!Overridelatched) {
    //   if (!climberLowerLimitOverride.get()) {
    //     elevatorClimbMotors[0].setSelectedSensorPosition(0);
    //     Overridelatched = true;
    //   } /*else if (!climberUpperLimitOverride.get()) {
    //       elevatorClimbMotors[0].setSelectedSensorPosition(Constants.Climber.climberUpperLimit);
    //       Overridelatched = true;
    //     }*/
    // } else if (Overridelatched && climberLowerLimitOverride.get()) {
    //   // && climberUpperLimitOverride.get()) {
    //   Overridelatched = false;
    // }
  }

  public void setClimberNeutralMode(NeutralMode mode) {
    elevatorClimbMotors[0].setNeutralMode(NeutralMode.Brake);
    elevatorClimbMotors[1].setNeutralMode(mode);
  }

  private void updateSmartDashboard() {
    // SmartDashboardTab.putBoolean(
    //     "Climber", "ClimberLowerOverride", climberLowerLimitOverride.get());
    // SmartDashboardTab.putBoolean(
    //     "Climber", "ClimberUpperOverride", climberUpperLimitOverride.get());
    // SmartDashboardTab.putBoolean(
    //     "Climber", "High Climb Piston Position", getHighClimbPistonPosition() == Value.kForward);
    // SmartDashboard.putBoolean("Climb Mode", getElevatorClimbState());
    // SmartDashboardTab.putNumber(
    //     "Climber", "Climb Output", elevatorClimbMotors[0].getMotorOutputPercent());
    // SmartDashboardTab.putNumber("Climber", "Climb Position", getElevatorClimbPosition());
    // SmartDashboard.putBoolean("Climber limit switch", climberLowerLimitOverride.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();

    if ((!climberLowerLimitOverride.get() && elevatorClimbMotors[0].getMotorOutputPercent() < 0)
        || (getElevatorClimbPosition() >= Constants.Climber.climberEncoderUpperLimit
            && elevatorClimbMotors[0].getMotorOutputPercent() > 0))
      elevatorClimbMotors[0].set(ControlMode.PercentOutput, 0);

    if (getElevatorClimbState()) updateClimberLimits();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
