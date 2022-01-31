// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * climber will only be making for a mid-climb as of 1/11/22 in the future I hope that the climber
 * will use some more advanced capability to get a traversal run climb in the future.
 */
public class Climber extends SubsystemBase {
  private final DoubleSolenoid climbBrakeSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, climbPistonForward, climbPistonReverse);
  private final TalonFX[] climbMotors = {new TalonFX(climbMotorA), new TalonFX(climbMotorB)};
  private boolean climbState;

  /** Creates a new Climber. */
  public Climber() {
    // Set up climber motor
    for (int i = 0; i < 1; i++) {
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
   * return the state of the climber brake
   *
   * @return the climber state the climb brake (true is engaged)
   */
  public ClimbBrakeStatus getClimbBrakeStatus() {
    return climbBrakeSolenoid.get() == DoubleSolenoid.Value.kForward
        ? ClimbBrakeStatus.ENGAGED
        : ClimbBrakeStatus.DISENGAGED;
  }

  /** sets the state of the climb piston */
  public void engagePistonBrake() {
    if (getClimbBrakeStatus() != ClimbBrakeStatus.ENGAGED) {
      climbBrakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  /** sets the state of the climb piston */
  public void disengagePistonBrake() {
    if (getClimbBrakeStatus() != ClimbBrakeStatus.DISENGAGED) {
      climbBrakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  /**
   * sets the climber motor's power with a percent (0.0 - 1.0)
   *
   * @param value output value
   */
  public void setClimberPercentOutput(double value) {
      climbMotors[0].set(ControlMode.PercentOutput, value);
  }

  /**
   * get the climber position
   *
   * @return the climber position (in raw sensor units)
   */
  public double getClimberPosition() {
    return climbMotors[0].getSelectedSensorPosition();
  }

  private void updateSmartDashboard() {
    SmartDashboardTab.putBoolean("Climber", "Climb Mode", getClimbState());
    SmartDashboardTab.putNumber("Climber", "Climb Output", climbMotors[0].getMotorOutputPercent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** The different states that the climber can be in */
  private enum ClimbBrakeStatus {
    ENGAGED,
    DISENGAGED
  }
}
