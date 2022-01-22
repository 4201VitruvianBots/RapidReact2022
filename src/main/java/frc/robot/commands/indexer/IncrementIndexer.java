// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/** An example command that uses an example subsystem. */
public class IncrementIndexer extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  double m_setpoint;

  private double startTime;

  public IncrementIndexer(Indexer indexer) {
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  /** Sets startTime to the Timer's value at the start */
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. Makes the Kicker spin
   * backwards
   */
  @Override
  public void execute() {
    m_indexer.setKickerOutput(-0.2);
  }

  /**
   * Called once the command ends or is interrupted. Sets the Kicker speed to 0 Gets the value of
   * how long the command has ran for
   */
  @Override
  public void end(boolean interrupted) {
    m_indexer.setKickerOutput(0);
    SmartDashboard.putNumber("Execution Time", Timer.getFPGATimestamp() - startTime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
