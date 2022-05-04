// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class DriveToVisionTarget extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final Vision m_vision;
  private final FieldSim m_fieldSim;
  private final DifferentialDriveOdometry m_tempOdometry;

  private RamseteController m_pathFollower = new RamseteController();
  private Trajectory m_path;
  DifferentialDriveWheelSpeeds m_prevSpeed = new DifferentialDriveWheelSpeeds();
  private Timer m_timer = new Timer();
  private double m_prevTime = 0;

  public DriveToVisionTarget(DriveTrain driveTrain, Vision vision, FieldSim fieldSim) {
    m_driveTrain = driveTrain;
    m_vision = vision;
    m_fieldSim = fieldSim;
    m_tempOdometry = new DifferentialDriveOdometry(m_driveTrain.getHeadingRotation2d());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.LIMELIGHT)) {
      m_tempOdometry.resetPosition(new Pose2d(), new Rotation2d());

      var goalOffset =
          new Transform2d(new Translation2d(-Units.feetToMeters(8), 0), new Rotation2d());
      var targetOffset =
          m_vision.getTransformFromTarget(Constants.Vision.CAMERA_POSITION.LIMELIGHT);

      var endPos = new Pose2d().plus(targetOffset).plus(goalOffset);

      TrajectoryConfig m_pathConfig =
          new TrajectoryConfig(8, 4)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(Constants.DriveTrain.kDriveKinematics);

      m_path =
          TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(), endPos, m_pathConfig);

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

      m_fieldSim.getField2d().getObject("DriveToTargetTrajectory").setPoses(trajectoryStates);

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
            m_pathFollower.calculate(m_tempOdometry.getPoseMeters(), currentReference);

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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
