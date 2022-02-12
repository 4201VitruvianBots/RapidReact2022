// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Controls;

public class SetFloodlight extends CommandBase {
  public final Controls m_controls;
  public boolean extend;

  /** Creates a new IntakePiston. */
  public SetFloodlight(Controls controls) {
    m_controls = controls;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(controls);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_controls.setPDHChannel(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controls.setPDHChannel(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
