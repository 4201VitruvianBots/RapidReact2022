// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * climber will only be making for a mid-climb as of 1/11/22 in the future I hope that the climber
 * will use some more advanced capability to get a traversal run climb in the future.
 */
public class Climber extends SubsystemBase {
  private final DoubleSolenoid climbPiston =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, climbPistonForward, climbPistonReverse);
  // TODO: Ask the design team about how the climber functions will work and if they will work
  // similarly or different than what the climber did on jango
  private final TalonFX climbMotor = new TalonFX(climbMotorA);
  private boolean climbState;

  /** Creates a new Climber. */
  public Climber() {
    // Set up climber motor
    this.climbMotor.configFactoryDefault();
    this.climbMotor.setSelectedSensorPosition(0);
    this.climbMotor.setNeutralMode(NeutralMode.Brake);
    this.climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  /**
   * return the extended state of the climber
   *
   * @return the climber state the piston (true is extended)
   */
  public boolean getClimbPistonExtendStatus() {
    return this.climbPiston.get() == DoubleSolenoid.Value.kForward;
  }

  /**
   * sets the state of the climb piston
   *
   * @param state true sets' climber to go up. false sets climber to go down
   */
  public void setClimbPiston(final boolean state) {
    this.climbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  /**
   * returns the state of the climb piston
   *
   * @return up is true. down is false
   */
  public boolean getClimbState() {
    return this.climbState;
  }

  /**
   * returns the state of the climb piston
   *
   * @param state up is true. down is false
   */
  public void setClimbState(final boolean state) {
    this.climbState = state;
  }

  /**
   * sets the climber motor's power with a percent (0.0 - 1.0)
   *
   * @param value output value
   */
  public void setClimberPercentOutput(final double value) {
    this.climbMotor.set(ControlMode.PercentOutput, value);
  }

  /**
   * get the climber position
   *
   * @return the climber position (in raw sensor units)
   */
  public double getClimberPosition() {
    return this.climbMotor.getSelectedSensorPosition();
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
