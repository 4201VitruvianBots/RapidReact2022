/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

/** Raises/lowers the climber based on joystick input */
public class EngageHighClimb extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Climber m_climber;

  private DoubleSupplier m_input;

  /**
   * Creates a new SetClimberOutput.
   *
   * @param climber The climber used by this command.
   * @param input The input used to control the climber output.
   */
  public EngageHighClimb(Climber climber) {
    m_climber = climber;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climber.getElevatorClimbState()) {
      m_climber.setHighClimbPiston(
          (m_climber.getHighClimbPistonPosition() == Value.kReverse
              ? Value.kForward
              : Value.kReverse));
    }
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_climber.setHighClimbPiston(Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  private enum climberState {
    MOVING,
    STILL
  }
}
