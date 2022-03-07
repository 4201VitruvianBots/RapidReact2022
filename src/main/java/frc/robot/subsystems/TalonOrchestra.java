// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TalonOrchestra extends SubsystemBase {

  private final TalonFX[] talons = {
      new TalonFX(Constants.Climber.climbMotorA),
      new TalonFX(Constants.Climber.climbMotorB),
      new TalonFX(Constants.Indexer.indexerMotor),
      new TalonFX(Constants.Indexer.kickerMotor),
      new TalonFX(Constants.Intake.intakeMotor),
      new TalonFX(Constants.DriveTrain.leftFrontDriveMotor),
      new TalonFX(Constants.DriveTrain.leftRearDriveMotor),
      new TalonFX(Constants.DriveTrain.rightFrontDriveMotor),
      new TalonFX(Constants.DriveTrain.rightRearDriveMotor),
      new TalonFX(Constants.Flywheel.flywheelMotorA),
      new TalonFX(Constants.Flywheel.flywheelMotorB),
      new TalonFX(Constants.Turret.turretMotor)
  };

  private Orchestra orchestra;

  public TalonOrchestra(DriveTrain driveTrain) {
    for (TalonFX talon : talons) {
      orchestra.addInstrument(talon);
    }
  }

  public void song(String name) {
    if (!orchestra.isPlaying()) {
      orchestra.loadMusic(name);
      orchestra.play();
    }
  }

  public void stopSong() {
    orchestra.stop();
  }

  @Override
  public void periodic() {
  }
}