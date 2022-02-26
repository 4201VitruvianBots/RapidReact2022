// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class TestLatency extends CommandBase {
  private DriveTrain m_driveTrain;
  private Vision m_vision;

  /** Creates a new TestLatency. */
  public TestLatency(DriveTrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_driveTrain = drivetrain;
  m_vision = vision; 
  addRequirements(drivetrain);
  addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_vision.getGoalValidTarget()){
      m_driveTrain.setMotorArcadeDrive(0.2 * Math.sin(Timer.getFPGATimestamp() * 1.5), 0);
    } else {
      m_driveTrain.setMotorArcadeDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setMotorArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
