// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Flywheel;

public class ColorSensor extends CommandBase {

  private final Indexer m_indexer;
  private final Intake m_intake;
  private final Controls m_controls;
  private final Flywheel m_flywheel;

  /** Creates a new ColorSensor. */
  public ColorSensor(Indexer indexer, Controls controls, Intake intake, Flywheel flywheel) {
    m_indexer = indexer;
    m_controls = controls;
    m_intake = intake;
    m_flywheel = flywheel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(intake);
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /** Uses intake if cargo color is wrong */
    if (m_indexer.getIndexerFrontSensorTripped()) {
      if (m_controls.getAllianceColor() != m_indexer.getCargoColor(Constants.Indexer.colorSensorFront)) {
        m_intake.setIntakePercentOutput(-0.8);
        m_indexer.setIndexerPercentOutput(-0.8);
      }
    } else {
      m_intake.setIntakePercentOutput(0);
      m_indexer.setIndexerPercentOutput(0);
    }
    
    /** Uses outtake if cargo color is wrong */
    if (m_indexer.getIndexerRearSensorTripped()) {
      if (m_controls.getAllianceColor() != m_indexer.getCargoColor(Constants.Indexer.colorSensorRear)) {
        m_indexer.setIndexerPercentOutput(0.5);
        m_indexer.setKickerPercentOutput(0.5);
        m_flywheel.setRPM(600);
      }
    } else {
      m_indexer.setIndexerPercentOutput(0);
      m_indexer.setKickerPercentOutput(0);
      m_flywheel.setRPM(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerPercentOutput(0);
    m_indexer.setKickerPercentOutput(0);
    m_flywheel.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
