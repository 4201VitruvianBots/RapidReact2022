package frc.robot.commands.simulation;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;

public class SetSimTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FieldSim m_fieldSim;

  private final Trajectory[] m_trajectories;

  /**
   * Sets the robot's position
   *
   * @param driveTrain Drivetrain's odometry is set
   * @param pose2d position to set odometry to
   */
  public SetSimTrajectory(FieldSim fieldSim, Trajectory... trajectories) {
    m_fieldSim = fieldSim;
    m_trajectories = trajectories;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_fieldSim.setAutoTrajectory(m_trajectories);
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
