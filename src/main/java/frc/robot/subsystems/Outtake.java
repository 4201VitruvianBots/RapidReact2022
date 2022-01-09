// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {
  /** Creates a new Outtake. */

  private final TalonFX[] outtakeMotors = {
    new TalonFX(Constants.Outtake.flywheelMotorA),
    new TalonFX(Constants.Outtake.flywheelMotorB)
  };
  public Outtake() {
      // Setup shooter motors (Falcons)
      for(TalonFX outtakeMotor : outtakeMotors) {
          outtakeMotor.configFactoryDefault();
          outtakeMotor.setNeutralMode(NeutralMode.Coast);
          outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
          outtakeMotor.configVoltageCompSaturation(10);
          outtakeMotor.enableVoltageCompensation(true);
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
