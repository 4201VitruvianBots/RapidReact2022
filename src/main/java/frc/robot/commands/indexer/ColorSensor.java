// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ColorSensor extends CommandBase {

  private final Indexer m_indexer;
  private final Intake m_intake;
  private final Controls m_controls;

  /** Creates a new ColorSensor. */
  public ColorSensor(Indexer indexer, Controls controls, Intake intake) {
    m_indexer = indexer;
    m_controls = controls;
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Uses intake if cargo color is wrong
     */
    if(m_indexer.getIndexerFrontSensorTripped()) {
      if(m_controls.getAllianceColor() != m_indexer.getCargoColor()) {
        new ReverseIntake(m_intake, m_indexer);
      } else if (m_controls.getAllianceColor() != m_indexer.getCargoColor()) {
        new ReverseIntake(m_intake, m_indexer);
      }
    }
    /**
     * Uses outtake if cargo color is wrong
     */
    if(m_indexer.getIndexerFrontSensorTripped()) {
      if(m_controls.getAllianceColor() != m_indexer.getCargoColor()) {
        //TODO: add outtake method
      } else if (m_controls.getAllianceColor() != m_indexer.getCargoColor()) {
        //TODO: add outtake method
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
