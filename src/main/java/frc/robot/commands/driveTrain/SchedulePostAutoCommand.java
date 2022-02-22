/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/** Schedules a {@link Command} to run upon teleop */
public class SchedulePostAutoCommand extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final Command m_command;

  /**
   * Schedules a {@link Command} to run upon teleop
   *
   * @param driveTrain The drivetrain used by this command
   * @param command The command to run
   */
  public SchedulePostAutoCommand(DriveTrain driveTrain, Command command) {
    m_driveTrain = driveTrain;
    m_command = command;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setPostAutoCommand(m_command);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
