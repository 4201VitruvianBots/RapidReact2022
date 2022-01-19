/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/** Raises/lowers the climber based on joystick input */
public class SetClimberOutput extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Climber m_climber;

  private final Joystick m_controller;

  private boolean currentDirection = true;
  private boolean movable, switchDirection;
  private double timestamp;
  private int lastDirection;

  /**
   * Creates a new SetClimberOutput.
   *
   * @param climber    The climber used by this command.
   * @param controller The joystick controller used by this command.
   */
  public SetClimberOutput(final Climber climber, final Joystick controller) {
    this.m_climber = climber;
    this.m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    final double input = Math.abs(this.m_controller.getRawAxis(5)) > 0.2 ? this.m_controller.getRawAxis(5) : 0;

    final int direction = input > 0 ? 1 : input < 0 ? -1 : 0;
    // might be better
    // if (this.m_climber.getClimbState()) {
    if (direction != this.lastDirection) {
      this.timestamp = Timer.getFPGATimestamp();
      this.movable = false;
      this.switchDirection = direction == 1;
    }

    if (this.movable) {
      final double output = input;
      this.m_climber.setClimberPercentOutput(output);
    } else {
      if (this.switchDirection)
        this.climberReleaseSequence();
      else {
        this.m_climber.setClimbPiston(false);
        this.movable = true;
        this.currentDirection = true;
      }
    }
    this.lastDirection = direction;
    // }
  }

  private void climberReleaseSequence() {
    this.m_climber.setClimbPiston(true);
    if (Math.abs(Timer.getFPGATimestamp() - this.timestamp) < 0.2)
      this.m_climber.setClimberPercentOutput(-0.35);
    else if (Math.abs(Timer.getFPGATimestamp() - this.timestamp) < 0.4)
      this.m_climber.setClimberPercentOutput(0.25);
    else {
      this.m_climber.setClimberPercentOutput(0);
      this.movable = true;
      this.currentDirection = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    this.m_climber.setClimberPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
