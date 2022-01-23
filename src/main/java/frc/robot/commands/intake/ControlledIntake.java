/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class ControlledIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final ColorSensors m_colorSensors;
  private final Intake m_intake;
  private final Joystick m_controller;

  private final double intakeRPM = 5000;
  private final double indexRPM = 300;

  private double timestamp, intakeTimestamp, indexerTimestamp, twoBallTimestamp;
  private boolean intaking, haveTwo, haveTwoTripped;
  private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;

  public enum IntakeStates {
    INTAKE_EMPTY,
    INTAKE_ONE_BALL,
    INTAKE_TWO_BALLS,
    INTAKE_THREE_BALLS
  //INTAKE_WRONG_BALL
  }

  public ControlledIntake(Intake intake, Indexer indexer, Joystick controller, ColorSensors colorSensors) {

    m_intake = intake;
    m_indexer = indexer;
    m_colorSensors = colorSensors;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
    addRequirements(colorSensors);
  }


  /**
   * Called when the command is initially scheduled. Sets the intake state to true Sets timestamp to
   * the same value as the timer
   */
  @Override
  public void initialize() {
    m_intake.setIntakeState(true);
    timestamp = Timer.getFPGATimestamp();
    
    if (m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor() && m_colorSensors.redMatch() || m_colorSensors.blueMatch())
     intakeState = IntakeStates.INTAKE_THREE_BALLS;
    else if (m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor() && m_colorSensors.redMatch() || m_colorSensors.blueMatch())
      intakeState = IntakeStates.INTAKE_TWO_BALLS;
    else if (m_colorSensors.redMatch() || m_colorSensors.blueMatch())
    intakeState = IntakeStates.INTAKE_EMPTY;
    else 
    return;
    
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. Spins the Intake forward
   * while keeping the Kicker and Indexer off
   */
  @Override
  public void execute() {
    switch(intakeState) {
      case INTAKE_THREE_BALLS:
          m_intake.setIntakePercentOutput(0);
          m_indexer.setKickerOutput(0);
          m_indexer.setIndexerOutput(0);
          m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
          m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
          break;
      case INTAKE_TWO_BALLS:
          m_intake.setIntakePercentOutput(0.8);
          m_indexer.setKickerOutput(0);
          if(m_indexer.getIntakeSensor())
              intakeState = IntakeStates.INTAKE_THREE_BALLS;
          break;
      case INTAKE_EMPTY:
      default:
          m_intake.setIntakePercentOutput(0.8);
          m_indexer.setKickerOutput(- 0.4);
          if(m_indexer.getIndexerBottomSensor()) {
              m_indexer.setIndexerOutput(0.95);
          } else {
              m_indexer.setIndexerOutput(0);
          }

          if(m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor()) {
            m_indexer.setRPM(0);
            intakeState = IntakeStates.INTAKE_TWO_BALLS;
            break;
      } 
    }
    


  }





  /**
   * If the Indexer time is not 0, and if the difference between the start time and indexer time is
   * less than 0.1, then we will reset the Indexer RPM If not, then we will set the Indexer RPM to 0
   * and Declare that the robot is not intaking
   */
  private void updateTimedRollers() {
    timestamp = Timer.getFPGATimestamp();
    if (indexerTimestamp != 0)
      if (timestamp - indexerTimestamp < 0.1) m_indexer.setRPM(indexRPM);
      else {
        m_indexer.setRPM(0);
        intaking = false;
      }
  }

  /**
   * Called once the command ends or is interrupted. We will set the Intake, Indexer, and kicker
   * speed to 0, turn off controller rumble, and also set the intake state to false
   */
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    m_intake.setIntakeState(false);
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerOutput(0);
    m_indexer.setKickerOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
