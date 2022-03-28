// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class ToggleColorSensorOverride extends CommandBase {

  private final Indexer m_indexer;

  /** Creates a new RunIndexer. */
  public ToggleColorSensorOverride(Indexer indexer) {
    m_indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.setColorSensorOverride(!m_indexer.getColorSensorOverride());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
