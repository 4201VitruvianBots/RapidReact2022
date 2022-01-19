// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class AutoControlledIntake extends CommandBase {
  /** Creates a new AutoControlledIntake. */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final Intake m_intake;

  private final double intakeRPM = 5000;
  private final double indexRPM = 300;
  private double timestamp, intakeTimestamp, indexerTimestamp;
  private boolean intaking;

  public AutoControlledIntake(Intake intake, Indexer indexer) {
   
    m_intake = intake;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeState(true);
    timestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakePercentOutput(1);
    m_indexer.setKickerOutput(0);
    m_indexer.setIndexerOutput(0);

  }

private void updateTimedRollers() {
  if(indexerTimestamp != 0)
    if(timestamp - indexerTimestamp < 0.1)
      m_indexer.setRPM(indexRPM);
    else {
      m_indexer.setRPM(0);
      intaking = false;
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeState(false);
        m_intake.setIntakePercentOutput(0);
        m_indexer.setIndexerOutput(0);
        m_indexer.setKickerOutput(0);
        m_intake.setIntakePiston(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
