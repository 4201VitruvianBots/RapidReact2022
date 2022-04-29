// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private boolean isIntaking = false; // is robot intaking

  // Intake motor setup

  private TalonFX[] intakeMotors = {
    new TalonFX(Constants.Intake.intakeMotor), new TalonFX(Constants.Intake.intakeRollerMotor)
  };

  // Intake piston setup
  DoubleSolenoid intakePiston =
      new DoubleSolenoid(
          Constants.Pneumatics.pcmOne,
          Constants.Pneumatics.pcmType,
          Constants.Pneumatics.intakePistonForward,
          Constants.Pneumatics.intakePistonReverse);

  public Intake() {
    // Motor configuration

    for (TalonFX intakeMotor : intakeMotors) {
      intakeMotor.configFactoryDefault();
      intakeMotor.setNeutralMode(NeutralMode.Coast);
      intakeMotor.configOpenloopRamp(0.5);
      intakeMotor.setStatusFramePeriod(1, 100);
      intakeMotor.setStatusFramePeriod(2, 100);
    }
    intakeMotors[0].setInverted(false);
    intakeMotors[1].setInverted(true);

    // SmartDashboard.putData("Intake Subsystem", this);
  }

  /** @return Gets a boolean for the intake's actuation */
  public boolean getIntakeState() {
    return isIntaking;
  }

  /** Sets a boolean for the intake's actuation */
  public void setIntakeState(boolean state) {
    isIntaking = state;
  }

  /** @return A boolean value based on the intake's piston status (up or down) */
  public boolean getIntakePistonExtendStatus() {
    return intakePiston.get() == DoubleSolenoid.Value.kForward;
  }

  /** Sets intake piston's states to forward and backward */
  public void setIntakePiston(boolean state) {
    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  /** sets the amount of power going to the intake */
  public void setIntakePercentOutput(double value) {
    intakeMotors[0].set(ControlMode.PercentOutput, value);
  }

  public void setIntakeRollerPercentOutput(double value) {
    intakeMotors[1].set(ControlMode.PercentOutput, value);
  }

  /** updates intake data on to the dashboard */
  public void updateSmartDashboard() {
    SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakeState());
    SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
    SmartDashboardTab.putNumber(
        "Intake",
        "Intake motor speed",
        intakeMotors[0].getSelectedSensorVelocity()
            * (10.0
                * 2.0
                * Math.PI
                / (Constants.Flywheel.encoderUnitsPerRotation
                    * Constants.Indexer.falconMaxSpeedRadPerSecond)));
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
}
