/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

/** Raises/lowers the climber based on joystick input */
public class SetClimbState extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Climber m_climber;

  private Intake m_intake;
  private boolean m_state;

  /**
   * Creates a new SetClimberOutput.
   *
   * @param climber The climber used by this command.
   * @param state The climb state to use
   */
  public SetClimbState(Climber climber, boolean state, Intake intake) {
    m_climber = climber;
    m_state = state;
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setElevatorClimbState(m_state);
  }

  @Override
  public void execute() {
    if (m_climber.getElevatorClimbState() == true) {
      m_intake.setIntakePiston(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
