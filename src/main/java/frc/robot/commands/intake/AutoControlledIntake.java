// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ColorSensors;

public class AutoControlledIntake extends CommandBase {
  /** Creates a new AutoControlledIntake. */
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;

  private final Intake m_intake;

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
  }

  public AutoControlledIntake(Intake intake, Indexer indexer) {

    m_intake = intake;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }

  /**
   * If the Intake, Indexer top and bottom sensors are one, there are 2 balls,
   * If the 
   * TODO: 
   */
  @Override
  public void initialize() {
    m_intake.setIntakeState(true);
    timestamp = Timer.getFPGATimestamp();
    if (m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor() /*&& TeamColor == TeamColorSet*/)
     intakeState = IntakeStates.INTAKE_THREE_BALLS;
    else if (m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_TWO_BALLS;
    else intakeState = IntakeStates.INTAKE_EMPTY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (intakeState) {
      case INTAKE_THREE_BALLS:
        m_intake.setIntakePercentOutput(0);
        m_indexer.setKickerOutput(0);
        m_indexer.setIndexerOutput(0);
        break;
      case INTAKE_TWO_BALLS:
        m_intake.setIntakePercentOutput(0.9);
        m_indexer.setKickerOutput(0);
        if (m_indexer.getIntakeSensor()) intakeState = IntakeStates.INTAKE_THREE_BALLS;

        break;
        case INTAKE_EMPTY:
      default:
        m_intake.setIntakePercentOutput(0.9);
        m_indexer.setKickerOutput(-0.4);
        if (m_indexer.getIndexerBottomSensor() && !intaking) {
          // indexerTimestamp = Timer.getFPGATimestamp();
          intaking = true;
          m_indexer.setIndexerOutput(0.95);
          //          m_indexer.setRPM(indexRPM);
        } else {
          intaking = false;
          m_indexer.setIndexerOutput(0);
          //          m_indexer.setRPM(0);
        }


        if (m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor()) {
          m_indexer.setRPM(0);
          intakeState = IntakeStates.INTAKE_TWO_BALLS;
        }
        break;
    }

    // updateTimedRollers();
  }

  private void updateTimedRollers() {
    timestamp = Timer.getFPGATimestamp();

    if (twoBallTimestamp != 0) haveTwo = (timestamp - twoBallTimestamp) > 0.5;

    if (intakeState != IntakeStates.INTAKE_EMPTY)
      if (indexerTimestamp != 0)
        if (timestamp - indexerTimestamp < 0.1) m_indexer.setRPM(indexRPM);
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
    if (intakeState == IntakeStates.INTAKE_THREE_BALLS) m_intake.setIntakePiston(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
