// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class RapidFireSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Flywheel m_flywheel;

  private final Indexer m_indexer;
  private final Intake m_intake;
  private double startTime, timestamp;
  private double timerStart;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RapidFireSetpoint(Flywheel flywheel, Indexer indexer, Intake intake) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_flywheel = flywheel;
    m_indexer = indexer;
    m_intake = intake;
    addRequirements(flywheel);
    addRequirements(indexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
}
//Uncomment execute once intake/indexer are made
// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
/*
    if(Math.abs(m_flywheel.getRPM(0) - m_flywheel.getSetpointRPM()) <= 100 || Timer.getFPGATimestamp() - startTime > 0.5) {
        m_indexer.setIndexerOutput(1);
        m_indexer.setKickerOutput(1);
        m_intake.setIntakePercentOutput(1);
    }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerOutput(0);
    m_indexer.setKickerOutput(0);
    m_flywheel.setPower(0);
*/}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return (false);
}
}
