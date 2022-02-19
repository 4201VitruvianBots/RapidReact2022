// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import java.util.function.Supplier;

public class ColorSensor extends CommandBase {

  private final Indexer m_indexer;
  private final Intake m_intake;
  private final Controls m_controls;
  private final Flywheel m_flywheel;
  private final Supplier<Boolean> m_triggerPressed;

  private DriverStation.Alliance allianceColor;

  private boolean frontCorrectColor = true;
  private boolean rearCorrectColor = true;

  private boolean frontTripped = false;
  private boolean rearTripped = false;

  /** Creates a new ColorSensor. */
  public ColorSensor(
      Indexer indexer,
      Controls controls,
      Intake intake,
      Flywheel flywheel,
      Supplier<Boolean> triggerPressed) {
    m_indexer = indexer;
    m_controls = controls;
    m_intake = intake;
    m_flywheel = flywheel;
    m_triggerPressed = triggerPressed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allianceColor = DriverStation.Alliance.Red;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!frontCorrectColor && rearTripped && !m_indexer.getIndexerRearSensorTripped()) {
      m_indexer.setIndexerPercentOutput(0.2);
      m_indexer.setKickerPercentOutput(0.5);
      m_flywheel.setRPM(600);
      frontTripped = m_indexer.getIndexerFrontSensorTripped();
      rearTripped = m_indexer.getIndexerRearSensorTripped();
      return;
    }

    frontTripped = m_indexer.getIndexerFrontSensorTripped();
    rearTripped = m_indexer.getIndexerRearSensorTripped();

    /** Uses outtake if cargo color is wrong */
    if (rearTripped) {
      if (frontTripped) {
        // Both rear and front cargo
        frontCorrectColor =
            (allianceColor == m_indexer.getFrontColorType()
                || m_indexer.getFrontColorType() == DriverStation.Alliance.Invalid);
        rearCorrectColor =
            (allianceColor == m_indexer.getRearColorType()
                || m_indexer.getRearColorType() == DriverStation.Alliance.Invalid);
        if (!rearCorrectColor) {
          if (!frontCorrectColor) {
            // none are right
            m_indexer.setIndexerPercentOutput(0.2);
            m_indexer.setKickerPercentOutput(0.5);
            m_flywheel.setRPM(600);
          } else {
            // Front is correct, rear is wrong
            m_indexer.setIndexerPercentOutput(0.2);
            m_indexer.setKickerPercentOutput(0.5);
            m_flywheel.setRPM(600);
          }
        } else {
          if (m_triggerPressed.get()) {
            m_indexer.setIndexerPercentOutput(0.4);
            m_indexer.setKickerPercentOutput(0.4);
          } else {
            m_indexer.setIndexerPercentOutput(0);
            m_indexer.setKickerPercentOutput(0);
            m_flywheel.setRPM(0);
          }
        }
      } else {
        // Only rear cargo
        rearCorrectColor =
            (allianceColor == m_indexer.getRearColorType()
                || m_indexer.getRearColorType() == DriverStation.Alliance.Invalid);
        if (!rearCorrectColor) {
          // Rear cargo is wrong
          m_indexer.setIndexerPercentOutput(0.2);
          m_indexer.setKickerPercentOutput(0.5);
          m_flywheel.setRPM(600);
        } else {
          if (m_triggerPressed.get()) {
            m_indexer.setIndexerPercentOutput(0.4);
            m_indexer.setKickerPercentOutput(0.4);
          } else {
            m_indexer.setIndexerPercentOutput(0);
            m_indexer.setKickerPercentOutput(0);
            m_flywheel.setRPM(0);
          }
        }
      }
    } else {
      if (m_indexer.getIndexerFrontSensorTripped()) {
        // Only front cargo
        frontCorrectColor =
            (allianceColor == m_indexer.getFrontColorType()
                || m_indexer.getFrontColorType() == DriverStation.Alliance.Invalid);
        if (!frontCorrectColor) {
          // Front cargo is wrong
          m_indexer.setIndexerPercentOutput(0.2);
          m_indexer.setKickerPercentOutput(0.5);
          m_flywheel.setRPM(600);
        } else {
          if (m_triggerPressed.get()) {
            m_indexer.setIndexerPercentOutput(0.4);
            m_indexer.setKickerPercentOutput(0.4);
          } else {
            m_indexer.setIndexerPercentOutput(0);
            m_indexer.setKickerPercentOutput(0);
            m_flywheel.setRPM(0);
          }
        }
      } else {
        if (m_triggerPressed.get()) {
          m_indexer.setIndexerPercentOutput(0.4);
          m_indexer.setKickerPercentOutput(0.4);
        } else {
          m_indexer.setIndexerPercentOutput(0);
          m_indexer.setKickerPercentOutput(0);
          m_flywheel.setRPM(0);
        }
      }
    }

    /* Front correct, rear wrong:
        Rev flywheel to low RPM, then increase the speed after we shoot
      Rear correct, front wrong:
        Rev up flywheel to full RPM, run indexer for a little bit to shoot, then slow down flywheel, then shoot wrong cargo at slow speed
      Front wrong, rear wrong:
        Rev up flywheel very slow, then rapid fire both cargo out
      Front correct, rear correct:
        Run flywheel normally (or let the operator do so)

      Only 1 ball, in the front, correct:
        Allow operators to run flywheel and index normally
      Only 1 ball, in the front, wrong:
        Run flywheel at low RPM, eject the cargo
      Only 1 ball, in the rear, correct:
        Allow operators to run flywheel and index normally
      Only 1 ball, in the rear, wrong:
        Run flywheel at low RPM, eject the cargo

      Questions:
        1. Do we want to control the flywheel and indexer at all times, or only when there is a wrong cargo, or just send a warning? Or maybe disable the shoot buttons?
        2. Should we have an LED state to indicate if there is a cargo of the wrong color?
    */

    SmartDashboardTab.putString("Indexer", "Alliance color", allianceColor.toString());

    SmartDashboardTab.putBoolean("Indexer", "Front correct", frontCorrectColor);
    SmartDashboardTab.putBoolean("Indexer", "Rear correct", rearCorrectColor);
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
