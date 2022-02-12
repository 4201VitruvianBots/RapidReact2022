// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class RunIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;

  private final Indexer m_indexer;

  /**
   * @param intake The intake used by this command
   * @param indexer The indexer used by this command
   */
  public RunIntake(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_intake.getIntakePistonExtendStatus())
      m_intake.setIntakePiston(true);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. Spins the Intake and
   * Indexer forward
   */
  @Override
  public void execute() {
    //  m_indexer.setIndexerPercentOutput(0.5);
    m_intake.setIntakePercentOutput(0.9);
    m_indexer.setIndexerPercentOutput(0.35);
  }

  /**
   * Called once the command ends or is interrupted. Sets the speed of the Intake and Indexer to 0
   */
  @Override
  public void end(boolean interrupted) {
    //  m_indexer.setIndexerPercentOutput(0);
    m_intake.setIntakePiston(false);
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
