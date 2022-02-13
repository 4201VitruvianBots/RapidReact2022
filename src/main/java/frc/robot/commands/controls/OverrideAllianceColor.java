// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Controls;

/** An example command that uses an example subsystem. */
public class OverrideAllianceColor extends CommandBase {
  private final Controls m_controls;
  private DriverStation.Alliance m_color;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OverrideAllianceColor(Controls controls, DriverStation.Alliance color) {
    m_controls = controls;
    m_color = color;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_controls);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controls.setOverrideFmsAlliance(true);
    m_controls.setOverrideFmsAllianceColor(m_color);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controls.setOverrideFmsAlliance(false);
    m_controls.setOverrideFmsAllianceColor(DriverStation.Alliance.Invalid);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
