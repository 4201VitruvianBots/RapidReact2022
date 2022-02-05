/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

/** Raises/lowers the climber based on joystick input */
public class SetClimberOutput extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Climber m_climber;

  private DoubleSupplier m_input;

  /**
   * Creates a new SetClimberOutput.
   *
   * @param climber The climber used by this command.
   * @param input The input used to control the climber output.
   */
  public SetClimberOutput(Climber climber, DoubleSupplier input) {
    m_climber = climber;
    m_input = input;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_climber.getClimbState()) {
      double input = Math.abs(m_input.getAsDouble()) > 0.2 ? -m_input.getAsDouble() * 0.25 : 0;

      climberState desiredDirection = ((input == 0) ? climberState.STILL : climberState.MOVING);
      switch (desiredDirection) {
        case MOVING:
          m_climber.disengagePistonBrake();
          m_climber.setClimberPercentOutput(input);
          break;
        case STILL:
        default:
          m_climber.setClimberPercentOutput(0);
          m_climber.engagePistonBrake();
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimberPercentOutput(0.0);
    m_climber.engagePistonBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private enum climberState {
    MOVING,
    STILL
  }
}
