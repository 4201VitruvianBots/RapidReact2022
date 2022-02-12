// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class RunIndexer extends CommandBase {

  private final Indexer m_indexer;
  private final Flywheel m_flywheel; 
  /** Creates a new RunIndexer. */
  public RunIndexer(Indexer indexer, Flywheel flywheel) {
    m_indexer = indexer;
    m_flywheel = flywheel; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.setIndexerPercentOutput(0.35);
    if(m_flywheel.canShoot()){
      m_indexer.setKickerPercentOutput(0.9);
    }
    else m_indexer.setKickerPercentOutput(0);
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
