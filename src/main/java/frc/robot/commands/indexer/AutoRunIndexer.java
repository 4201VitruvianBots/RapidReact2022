// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class AutoRunIndexer extends CommandBase {

  private final Indexer m_indexer;
  private final Flywheel m_flywheel;
  private final boolean m_reverseIndexer;
  private final double m_kickerOutput;
  /** Creates a new RunIndexer. */
  public AutoRunIndexer(Indexer indexer, Flywheel flywheel) {
    this(indexer, flywheel, 0.85);
  }

  public AutoRunIndexer(Indexer indexer, Flywheel flywheel, double kickerOutput) {
    this(indexer, flywheel, kickerOutput, false);
  }

  public AutoRunIndexer(
      Indexer indexer, Flywheel flywheel, double kickerOutput, boolean reverseIndexer) {
    m_indexer = indexer;
    m_flywheel = flywheel;
    m_kickerOutput = kickerOutput;
    m_reverseIndexer = reverseIndexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_flywheel.canShoot()) m_indexer.setKickerPercentOutput(0.85);
    m_indexer.setKickerPercentOutput(m_kickerOutput);

    m_indexer.setIndexerPercentOutput(m_reverseIndexer ? -0.55 : 0.55);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.setIndexerPercentOutput(0);
    m_indexer.setKickerPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
