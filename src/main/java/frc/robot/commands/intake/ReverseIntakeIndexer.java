// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class ReverseIntakeIndexer extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;

  private final Indexer m_indexer;
  private final double m_intakeSpeed;

  public ReverseIntakeIndexer(Intake intake, Indexer indexer, double intakeSpeed) {
    m_intake = intake;
    m_indexer = indexer;
    m_intakeSpeed = intakeSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }

  /**
   * @param intake The intake used by this command
   * @param indexer The indexer used by this command
   */
  public ReverseIntakeIndexer(Intake intake, Indexer indexer) {
    this(intake, indexer, -0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakePiston(true);
    m_intake.setIntakeState(true);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. Spins the Intake and
   * Indexer forward
   */
  @Override
  public void execute() {
    m_intake.setIntakePercentOutput(m_intakeSpeed);
    m_indexer.setIndexerPercentOutput(-0.65);
    m_indexer.setKickerPercentOutput(-0.35);
  }

  /**
   * Called once the command ends or is interrupted. Sets the speed of the Intake and Indexer to 0
   */
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerPercentOutput(0);
    m_indexer.setKickerPercentOutput(0);
    m_intake.setIntakePiston(false);
    m_intake.setIntakeState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
