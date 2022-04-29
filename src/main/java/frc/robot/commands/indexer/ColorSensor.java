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
import frc.robot.commands.flywheel.ShotSelecter;

public class ColorSensor extends CommandBase {

  private final Indexer m_indexer;
  private final Intake m_intake;
  private final Controls m_controls;
  private final Flywheel m_flywheel;
  private final Supplier<Boolean> m_lTriggerHeld;
  // private final Supplier<Boolean> m_rTriggerHeld;
  // private final Supplier<Boolean> m_XboxButton0;
  // private final Supplier<Boolean> m_XboxButton1;
  // private final Supplier<Boolean> m_XboxButton3;

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
      Supplier<Boolean> lTriggerHeld) {
    // Supplier<Boolean> xboxButton0,
    // Supplier<Boolean> xboxButton1,
    // Supplier<Boolean> xboxButton3,
    // Supplier<Boolean> rTriggerHeld) {
    m_indexer = indexer;
    m_controls = controls;
    m_intake = intake;
    m_flywheel = flywheel;
    m_lTriggerHeld = lTriggerHeld;
    // m_rTriggerHeld = rTriggerHeld;
    // m_xboxButton0 = xboxButton0;
    // m_xboxButton1 = xboxButton1;
    // m_xboxButton3 = xboxButton3;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    allianceColor = DriverStation.getAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!frontCorrectColor && rearTripped && !m_indexer.getIndexerRearSensorTripped()) {
    //   m_indexer.setIndexerPercentOutput(0.2);
    //   m_indexer.setKickerPercentOutput(0.5);
    //   m_flywheel.setRPM(600);
    //   frontTripped = m_indexer.getIndexerFrontSensorTripped();
    //   rearTripped = m_indexer.getIndexerRearSensorTripped();
    //   return;
    // }

    if (m_indexer.getColorSensorOverride()) {
      if (m_lTriggerHeld.get()) {
        m_intake.setIntakePercentOutput(0.4);

      }

      // if (m_xboxButton0.get()) {
      //   m_flywheel.setRPM(900);
      // }
      // if (m_xboxButton1.get()) {
      //   m_flywheel.setRPM(1800);
      // }
      // if (m_xboxButton3.get()) {
      //   m_flywheel.setRPM(2650);
      // }

      // if (m_rTriggerHeld.get()) {
      //   m_indexer.setIndexerPercentOutput(0.6);
      // m_indexer.setKickerPercentOutput(0.85);
      //  }
      else {
        m_intake.setIntakePercentOutput(0);
        m_indexer.setIndexerPercentOutput(0);
        m_indexer.setKickerPercentOutput(0);
        m_flywheel.setRPM(0);
      }
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

        // Rear
        if (!rearCorrectColor) {
          if (!frontCorrectColor) {
            // none are right
            m_indexer.setIndexerPercentOutput(0.5);
            m_indexer.setKickerPercentOutput(0.5);
            m_flywheel.setRPM(m_flywheel.getInterpulatedRPM() + 500);
          } else {
            // Front is correct, rear is wrong
            m_indexer.setIndexerPercentOutput(0.5);
            m_indexer.setKickerPercentOutput(0.5);
            m_flywheel.setRPM(m_flywheel.getInterpulatedRPM() + 500);
          }
        }

        // front is wrong, rear is correct
        if (rearCorrectColor) {
          if (!frontCorrectColor) {

          } else {
            if (m_lTriggerHeld.get()) {
              m_intake.setIntakePercentOutput(0.4);
            }

            // if (m_xboxButton0.get()) {
            //   m_flywheel.setRPM(900);
            // }
            // if (m_xboxButton1.get()) {
            //   m_flywheel.setRPM(1800);
            // }
            // if (m_xboxButton3.get()) {
            //   m_flywheel.setRPM(2650);
            // }

            // if (m_rTriggerHeld.get()) {
            //   m_indexer.setIndexerPercentOutput(0.6);
            // m_indexer.setKickerPercentOutput(0.85);
            //  }
            else {
              m_indexer.setIndexerPercentOutput(0);
              m_indexer.setKickerPercentOutput(0);
              m_intake.setIntakePercentOutput(0);
              m_flywheel.setRPM(0);
            }
          }
        }
      } else {
        // Only rear cargo
        rearCorrectColor =
            (allianceColor == m_indexer.getRearColorType()
                || m_indexer.getRearColorType() == DriverStation.Alliance.Invalid);
        if (!rearCorrectColor) {
          // Rear cargo is wrong
          m_indexer.setIndexerPercentOutput(0.5);
          m_indexer.setKickerPercentOutput(0.5);
          m_flywheel.setRPM(m_flywheel.getInterpulatedRPM() + 500);
        } else {
          if (m_lTriggerHeld.get()) {
            m_intake.setIntakePercentOutput(0.4);

            // if (m_xboxButton0.get()) {
            //   m_flywheel.setRPM(900);
            // }
            // if (m_xboxButton1.get()) {
            //   m_flywheel.setRPM(1800);
            // }
            // if (m_xboxButton3.get()) {
            //   m_flywheel.setRPM(2650);
            // }

            // if (m_rTriggerHeld.get()) {
            //   m_indexer.setIndexerPercentOutput(0.6);
            // m_indexer.setKickerPercentOutput(0.85);
            //  }
          } else {
            m_indexer.setIndexerPercentOutput(0);
            m_indexer.setKickerPercentOutput(0);
            m_flywheel.setRPM(0);
            m_intake.setIntakePercentOutput(0);
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
          m_indexer.setIndexerPercentOutput(0.5);
          m_indexer.setKickerPercentOutput(0.5);
          m_flywheel.setRPM(m_flywheel.getInterpulatedRPM() + 500);
          //add withTimeout
        } else {
          if (m_lTriggerHeld.get()) {
            m_intake.setIntakePercentOutput(0.4);
          }

          // if (m_xboxButton0.get()) {
          //   m_flywheel.setRPM(900);
          // }
          // if (m_xboxButton1.get()) {
          //   m_flywheel.setRPM(1800);
          // }
          // if (m_xboxButton3.get()) {
          //   m_flywheel.setRPM(2650);
          // }

          // if (m_rTriggerHeld.get()) {
          //   m_indexer.setIndexerPercentOutput(0.6);
          // m_indexer.setKickerPercentOutput(0.85);
          //  }
          else {
            m_indexer.setIndexerPercentOutput(0);
            m_indexer.setKickerPercentOutput(0);
            m_intake.setIntakePercentOutput(0);
            m_flywheel.setRPM(0);
          }
        }
      } else {
        if (m_lTriggerHeld.get()) {
          m_intake.setIntakePercentOutput(0.4);
        }

        // if (m_xboxButton0.get()) {
        //   m_flywheel.setRPM(900);
        // }
        // if (m_xboxButton1.get()) {
        //   m_flywheel.setRPM(1800);
        // }
        // if (m_xboxButton3.get()) {
        //   m_flywheel.setRPM(2650);
        // }

        // if (m_rTriggerHeld.get()) {
        //   m_indexer.setIndexerPercentOutput(0.6);
        // m_indexer.setKickerPercentOutput(0.85);
        //  }
        else {
          m_indexer.setIndexerPercentOutput(0);
          m_indexer.setKickerPercentOutput(0);
          m_intake.setIntakePercentOutput(0);
          m_flywheel.setRPM(0);
          m_flywheel.setRPM(m_flywheel.getInterpulatedRPM());
        }
      }
    }


    //TODO: add m_flywheel.setRPM(m_flywheel.getInterpulatedRPM(); to all else statements to revert speed
    /*

          Default Speeds for Color Sensor [

          * 40% speed for Intake,

          * 50% speed for Kicker and Indexer

          * 600 RPM for Flywheel
    ]

          Default Speeds for Suppliers[

          * 40% speed for Intake,

          * 60% speed for Indexer

          * 85% speed for Kicker

          *  900 RPM for Flywheel button 0

          *  1800 RPM for Flywheel button 1

          *  2650 RPM for Flywheel button 3
    ]

            TWO BALLS[
              Front correct, rear wrong:
                Rev flywheel to low RPM, then increase the speed after we shoot (line: 93)

              Front wrong, Rear correct:
                Eject bad cargo through the intake, and move the good cargo at the front. (line: 100)

              Front wrong, Rear wrong:
                Rev up flywheel very slow, then rapid fire both cargo out (line: 88)

              Front correct, rear correct:
                Run flywheel normally (or let the operator do so) (No line needed)
        ]


            ONE BALL[
              Front correct:
                Allow operators to run flywheel and index normally (No line needed)

              Rear correct:
                Allow operators to run flywheel and index normally (No line needed)

              Front wrong:
                Reverse the Intake (line: 147)

              Rear wrong:
                Run flywheel at low RPM, eject the cargo (line: 128)

              Questions:
                1. Do we want to control the flywheel and indexer at all times, or only when there is a wrong cargo, or just send a warning? Or maybe disable the shoot buttons?
                    A: :We are going to have it so that the driver can not shoot while the color sensors are running. There is an override button that just in case the color sensor stops functioning, we can stop it mid game.


                2. Should we have an LED state to indicate if there is a cargo of the wrong color?
                    A: For now, Jet says no, could be a smart idea in the future, but since it already ejects it, there is not too much of a point
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
