// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.CAMERA_POSITION;
import frc.robot.commands.auto.VitruvianRamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class DriveToVisionTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final Vision m_vision;

  private TrajectoryConfig m_pathConfig;
  private Trajectory m_path, projectedPath;
  private ArrayList<Pose2d> trajectoryStates;
  private VitruvianRamseteCommand m_command;

  private Pose2d startPos, endPos;
  private Translation2d visionTargetPose;
  private Rotation2d endAngle;

  private boolean finished = false;

  /** Creates a new ExampleCommand. */
  public DriveToVisionTarget(DriveTrain driveTrain, Vision vision) {
    m_driveTrain = driveTrain;
    m_vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;

    startPos = m_driveTrain.getRobotPoseMeters();

    if (m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.LIMELIGHT)) {
      visionTargetPose =
          m_vision
              .getHubPositionFromRobot()
              .rotateBy(
                  new Rotation2d(
                      Units.degreesToRadians(m_vision.getTargetYAngle(CAMERA_POSITION.LIMELIGHT))))
              .plus(
                  new Translation2d(
                      Constants.Vision.LOWER_HUB_RADIUS_METERS + Units.feetToMeters(1), 0))
              .rotateBy(startPos.getRotation())
              .plus(startPos.getTranslation());
      Rotation2d angletoVisionTarget =
          new Rotation2d(
              visionTargetPose.getX() - startPos.getX(), visionTargetPose.getY() - startPos.getY());
      endAngle = angletoVisionTarget;
      endPos = new Pose2d(visionTargetPose, endAngle);
    } else {
      finished = true;
      return;
    }

    m_pathConfig =
        new TrajectoryConfig(Units.feetToMeters(5), Units.feetToMeters(2.5))
            // Add kinematics to ensure max speed is actually obeyed
            .setReversed(false)
            .setKinematics(Constants.DriveTrain.kDriveKinematics)
            .setEndVelocity(0)
            .setStartVelocity(0);

    m_path = TrajectoryGenerator.generateTrajectory(startPos, List.of(), endPos, m_pathConfig);

    projectedPath =
        m_path.transformBy(
            new Transform2d(
                m_driveTrain.getRobotPoseMeters().getTranslation(),
                new Rotation2d(Units.degreesToRadians(m_driveTrain.getHeadingDegrees()))));

    trajectoryStates = new ArrayList<Pose2d>();
    trajectoryStates.addAll(
        projectedPath.getStates().stream()
            .map(state -> state.poseMeters)
            .collect(Collectors.toList()));

    m_driveTrain.setCurrentTrajectory(m_path);

    m_command = TrajectoryUtils.generateRamseteCommand(m_driveTrain, m_path);
    m_command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!finished) m_command.execute();
    finished = m_command.isFinished();
    SmartDashboard.putBoolean("Drive To Vision Target command finished", m_command.isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setVoltageOutput(0, 0);

    if (m_command != null) m_command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
