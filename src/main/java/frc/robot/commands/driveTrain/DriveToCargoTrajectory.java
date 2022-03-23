package frc.robot.commands.driveTrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class DriveToCargoTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final Vision m_vision;

  private RamseteController m_pathFollower = new RamseteController();
  private Trajectory m_path;
  DifferentialDriveWheelSpeeds m_prevSpeed = new DifferentialDriveWheelSpeeds();
  private Timer m_timer = new Timer();
  private double m_prevTime = 0;
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
      var endPos =
          new Pose2d(
              m_vision.getCargoPositionFromRobot().getTranslation(),
              m_driveTrain.getHeadingRotation2d()); // TODO: How to determine best heading?

      TrajectoryConfig m_pathConfig =
          new TrajectoryConfig(Units.feetToMeters(14), Units.feetToMeters(5))
              // Add kinematics to ensure max speed is actually obeyed
              .setReversed(true)
              .setKinematics(Constants.DriveTrain.kDriveKinematics)
              .setEndVelocity(3)
              .setStartVelocity(
                  (m_driveTrain.getSpeedsMetersPerSecond().leftMetersPerSecond
                          + m_driveTrain.getSpeedsMetersPerSecond().rightMetersPerSecond)
                      / 2);
              

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

      m_timer.reset();
      m_timer.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = m_timer.get();

    double dt = currentTime - m_prevTime;
    m_prevTime = currentTime;

    if (dt > 0) {
      try {
        var currentReference = m_path.sample(currentTime);

        var m_chassisSpeeds =
            m_pathFollower.calculate(m_driveTrain.getRobotPoseMeters(), currentReference);

        var targetSpeeds = m_driveTrain.getDriveTrainKinematics().toWheelSpeeds(m_chassisSpeeds);

        double leftTargetSpeed = targetSpeeds.leftMetersPerSecond;
        double rightTargetSpeed = targetSpeeds.rightMetersPerSecond;

        double leftOutput =
            m_driveTrain
                .getLeftPIDController()
                .calculate(
                    m_driveTrain.getSpeedsMetersPerSecond().leftMetersPerSecond, leftTargetSpeed);
        leftOutput +=
            m_driveTrain
                .getFeedforward()
                .calculate(
                    leftTargetSpeed, (leftTargetSpeed - m_prevSpeed.leftMetersPerSecond) / dt);

        double rightOutput =
            m_driveTrain
                .getLeftPIDController()
                .calculate(
                    m_driveTrain.getSpeedsMetersPerSecond().rightMetersPerSecond, rightTargetSpeed);
        rightOutput +=
            m_driveTrain
                .getFeedforward()
                .calculate(
                    rightTargetSpeed, (rightTargetSpeed - m_prevSpeed.rightMetersPerSecond) / dt);

        m_driveTrain.setVoltageOutput(leftOutput, rightOutput);
      } catch (Exception e) {

      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setVoltageOutput(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pathFollower.atReference();
  }
}
