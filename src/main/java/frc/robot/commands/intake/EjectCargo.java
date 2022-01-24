// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class EjectCargo extends CommandBase {
  /** Creates a new EjectCargo. */
  private final Intake m_intake;
  private final Indexer m_indexer;

  public EjectCargo(Intake intake, Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.    m_intake = intake;
    m_indexer = indexer;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //TODO: Use ColorSensors to detect if a there is a bad ball, if so, reverse until there is a correct ball
  @Override
  public void execute() {
    m_indexer.setIndexerPercentOutput(-0.8);
    m_intake.setIntakePercentOutput(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.setIndexerPercentOutput(0);
    m_intake.setIntakePercentOutput(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
