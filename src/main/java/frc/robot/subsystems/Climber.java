// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * climber will only be making for a mid-climb as of 1/11/22 in the future I hope that the climber
 * will use some more advanced capability to get a traversal run climb in the future.
 */
public class Climber extends SubsystemBase {
  public final TalonFX[] elevatorClimbMotors = {
    new TalonFX(Constants.Climber.climbMotorA), new TalonFX(Constants.Climber.climbMotorB),
  };

  DoubleSolenoid highClimbPiston =
      new DoubleSolenoid(
          Constants.Pneumatics.pcmOne,
          PneumaticsModuleType.CTREPCM,
          Constants.Pneumatics.climbPistonForward,
          Constants.Pneumatics.climbPistonReverse);

  private boolean elevatorClimbState;

  private final double elevatorClimbUpperLimit = 205_000.0;
  private final double elevatorClimbLowerLimit = 0.0;

  /** Creates a new Climber. */
  public Climber() {
    // Set up climber motor
    for (int i = 0; i < elevatorClimbMotors.length; i++) {
      elevatorClimbMotors[i].configFactoryDefault();
      elevatorClimbMotors[i].setSelectedSensorPosition(0);
      elevatorClimbMotors[i].setNeutralMode(NeutralMode.Brake);
      elevatorClimbMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    elevatorClimbMotors[1].set(TalonFXControlMode.Follower, elevatorClimbMotors[0].getDeviceID());
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
    if ((getElevatorClimbPosition() > elevatorClimbLowerLimit || value > 0)
        && (getElevatorClimbPosition() < elevatorClimbUpperLimit || value < 0))
      elevatorClimbMotors[0].set(ControlMode.PercentOutput, value);
    else elevatorClimbMotors[0].set(ControlMode.PercentOutput, 0);
  }

  public void setHighClimbPiston(DoubleSolenoid.Value kValue) {
    highClimbPiston.set(kValue);
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

  private void updateSmartDashboard() {
    SmartDashboardTab.putBoolean(
        "Climber",
        "High Climb Piston Position",
        getHighClimbPistonPosition() == Value.kForward ? true : false);
    SmartDashboardTab.putBoolean("Climber", "Climb Mode", getElevatorClimbState());
    SmartDashboardTab.putNumber(
        "Climber", "Climb Output", elevatorClimbMotors[0].getMotorOutputPercent());
    SmartDashboardTab.putNumber("Climber", "Climb Position", getElevatorClimbPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();

    if ((getElevatorClimbPosition() <= elevatorClimbLowerLimit
            && elevatorClimbMotors[0].getMotorOutputPercent() < 0)
        || (getElevatorClimbPosition() >= elevatorClimbUpperLimit
            && elevatorClimbMotors[0].getMotorOutputPercent() > 0))
      elevatorClimbMotors[0].set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
