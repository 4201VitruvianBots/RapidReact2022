// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/** An example command that uses an example subsystem. */
public class FeedAll extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FeedAll(Indexer indexer) {
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled. Spins the Indexer and the
   * Kicker forward
   */
  @Override
  public void execute() {
    m_indexer.setIndexerOutput(0.6);
    m_indexer.setKickerOutput(0.5);
  }

  /** Called once the command ends or is interrupted. Sets the Indexer and Kicker to a speed of 0 */
  @Override
  public void end(boolean interrupted) {
    m_indexer.setKickerOutput(0);
    m_indexer.setIndexerOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double time = Timer.getFPGATimestamp();
    if (m_indexer.getIndexerSensor()) {
      time = Timer.getFPGATimestamp();
    }
    return time >= 2;
  }
}
