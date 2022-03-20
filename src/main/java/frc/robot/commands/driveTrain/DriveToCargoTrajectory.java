package frc.robot.commands.driveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.auto.VitruvianRamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class DriveToCargoTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final Vision m_vision;

  private Trajectory m_path;
  DifferentialDriveWheelSpeeds m_prevSpeed = new DifferentialDriveWheelSpeeds();
  VitruvianRamseteCommand m_command;

  /** Creates a new ExampleCommand. */
  public DriveToCargoTrajectory(DriveTrain driveTrain, Vision vision) {
    m_driveTrain = driveTrain;
    m_vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.INTAKE)) {
      var startPos = m_driveTrain.getRobotPoseMeters();
      Translation2d cargoPose = m_vision.getCargoPositionFromRobot().getTranslation();
      Rotation2d angleToCargo =
          new Rotation2d(
              Math.atan2(cargoPose.getY() - startPos.getY(), cargoPose.getX() - startPos.getX()));
      var endPos =
          new Pose2d(cargoPose, angleToCargo.minus(startPos.getRotation().minus(angleToCargo)));

      TrajectoryConfig m_pathConfig =
          new TrajectoryConfig(Units.feetToMeters(14), Units.feetToMeters(5))
              // Add kinematics to ensure max speed is actually obeyed
              .setReversed(true)
              .setKinematics(Constants.DriveTrain.kDriveKinematics)
              .setEndVelocity(0)
              .setStartVelocity(0);

      m_path = TrajectoryGenerator.generateTrajectory(startPos, List.of(), endPos, m_pathConfig);

      var projectedPath =
          m_path.transformBy(
              new Transform2d(
                  m_driveTrain.getRobotPoseMeters().getTranslation(),
                  new Rotation2d(Units.degreesToRadians(m_driveTrain.getHeadingDegrees()))));

      var trajectoryStates = new ArrayList<Pose2d>();
      trajectoryStates.addAll(
          projectedPath.getStates().stream()
              .map(state -> state.poseMeters)
              .collect(Collectors.toList()));

      m_driveTrain.setCurrentTrajectory(m_path);

      m_command = TrajectoryUtils.generateRamseteCommand(m_driveTrain, m_path);
      m_command.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setVoltageOutput(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished(); // m_pathFollower.atReference();
  }
}
