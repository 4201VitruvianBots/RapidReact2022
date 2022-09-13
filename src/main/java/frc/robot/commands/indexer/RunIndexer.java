// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RunIndexer extends CommandBase {
  private final Indexer m_indexer;
  private final boolean m_runKicker;

  /** Creates a new RunIndexer. */
  public RunIndexer(Intake intake, Indexer indexer, Flywheel flywheel, boolean runKicker) {
    m_indexer = indexer;
    m_runKicker = runKicker;

    // Use addRequirements() here to declare subsystem dependencies.
    if (!runKicker) addRequirements(intake);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_runKicker) m_indexer.setKickerPercentOutput(0.8);

    m_indexer.setIndexerPercentOutput(0.65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.setIndexerPercentOutput(0);
    if (m_runKicker) m_indexer.setKickerPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
