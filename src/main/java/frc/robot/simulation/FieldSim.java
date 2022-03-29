package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimConstants.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

public class FieldSim {
  private final Field2d m_field2d;
  private final DriveTrain m_driveTrain;
  private final Turret m_turret;
  private final Vision m_vision;
  private final Intake m_intake;
  private ArrayList<Pose2d> trajectoryPoses;
  private Translation2d[] intakePositions = {
    new Translation2d(-SimConstants.robotLengthMeters / 2.0, SimConstants.robotWidthMeters / 2.0),
    new Translation2d(-SimConstants.robotLengthMeters / 2.0, -SimConstants.robotWidthMeters / 2.0),
    new Translation2d(
        -(SimConstants.robotLengthMeters / 2.0) - SimConstants.intakeLengthMeters,
        SimConstants.robotWidthMeters / 2.0),
    new Translation2d(
        -(SimConstants.robotLengthMeters / 2.0) - SimConstants.intakeLengthMeters,
        -SimConstants.robotWidthMeters / 2.0),
  };
  private double slope0to1 = 0;
  private double slope1to2 = 0;
  private Translation2d intakePoseFromChassis;
  private Pose2d robotPose;
  private Cargo[] cargo;
  private Pose2d ballPose;
  private double currentTime = 0;

  private double deltaT = 0;
  private double distanceTraveled = 0;

  private double deltaX = 0;
  private double deltaY = 0;
  // Exclude 5 cargo in other robots and 2 with human players
  private final Cargo[] m_redCargo = new Cargo[11];
  private final Cargo[] m_blueCargo = new Cargo[11];

  private int ballCount = 0;
  private final Pose2d[] intakePose = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
  private final Pose2d intakePoseHidden = new Pose2d(-50, 50, new Rotation2d());

  public FieldSim(DriveTrain driveTrain, Turret turret, Vision vision, Intake intake) {
    m_driveTrain = driveTrain;
    m_turret = turret;
    m_vision = vision;
    m_intake = intake;

    m_field2d = new Field2d();
    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void initSim() {
    // Places and sets the position of all the cargo on the field

    for (int i = 0; i < m_blueCargo.length; i++) {
      if (i < SimConstants.blueHubBallPos.length) {
        m_blueCargo[i] =
            new Cargo(
                String.format("BlueCargo_" + (i)), BallColor.BLUE, SimConstants.blueHubBallPos[i]);
        m_blueCargo[i].setBallState(BallState.ON_FIELD);
      } else {
        m_blueCargo[i] = new Cargo(String.format("BlueCargo_" + (i)), BallColor.BLUE, new Pose2d());
        m_blueCargo[i].setBallState(BallState.ON_FIELD);
      }
    }

    for (int i = 0; i < m_redCargo.length; i++) {
      if (i < SimConstants.redHubBallPos.length) {
        m_redCargo[i] =
            new Cargo(String.format("RedCargo_" + i), BallColor.RED, SimConstants.redHubBallPos[i]);
        m_redCargo[i].setBallState(BallState.ON_FIELD);
      } else {
        m_redCargo[i] = new Cargo(String.format("RedCargo_" + i), BallColor.RED, new Pose2d());
        m_redCargo[i].setBallState(BallState.ON_FIELD);
      }
    }

    // Loads 1 cargo into the robot
    m_blueCargo[6].setBallState(BallState.IN_ROBOT);

    m_field2d.setRobotPose(SimConstants.startPositionMeters);
    m_driveTrain.resetOdometry(
        SimConstants.startPositionMeters, SimConstants.startPositionMeters.getRotation());
  }

  public void setAutoTrajectory(Trajectory... trajectories) {
    trajectoryPoses = new ArrayList<Pose2d>();

    for (int i = 0; i < trajectories.length; i++) {
      trajectoryPoses.addAll(
          trajectories[i].getStates().stream()
              .map(state -> state.poseMeters)
              .collect(Collectors.toList()));
    }
  }

  public void clearAutoTrajectory() {
    trajectoryPoses = new ArrayList<Pose2d>();

    //    m_field2d.getObject("trajectory").setPoses(trajectoryPoses);
  }

  private void updateIntakePoses() {
    /* Intake Points:
      ^: Front of the robot
         -------
        |   ^   |
        |       |
        0-------1
        |       |
        3-------2
    */

    // Look up rotating a point about another point in 2D space for the math explanation

    robotPose = m_driveTrain.getRobotPoseMeters();
    for (int i = 0; i < intakePose.length; i++) {
      intakePoseFromChassis =
          intakePositions[i].rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
      intakePose[i] = new Pose2d(intakePoseFromChassis, robotPose.getRotation());
    }
  }

  private boolean isBallInIntakeZone(Pose2d ballPose) {
    // The rise/run between intake points 0 to 1
    // Since the intake is a rectangle, this is the same as the slope between points 2 to 3
    slope0to1 =
        (intakePose[1].getY() - intakePose[0].getY())
            / (intakePose[1].getX() - intakePose[0].getX());

    // The rise/run between points 1 to 2
    // Same as slope between points 3 and 0
    slope1to2 =
        (intakePose[2].getY() - intakePose[1].getY())
            / (intakePose[2].getX() - intakePose[1].getX());

    // Use point-slope form to check if ball pose is above or below each line on the intake
    // rectangle
    // For each pair of parallel lines, the ball needs to be above one line and below the other
    // Note: it's very important that the points be in the same order as the diagram above
    return ((ballPose.getY()
                >= slope0to1 * (ballPose.getX() - intakePose[0].getX()) + intakePose[0].getY())
            == (ballPose.getY()
                <= slope0to1 * (ballPose.getX() - intakePose[2].getX()) + intakePose[2].getY()))
        && ((ballPose.getY()
                >= slope1to2 * (ballPose.getX() - intakePose[0].getX()) + intakePose[0].getY())
            == (ballPose.getY()
                <= slope1to2 * (ballPose.getX() - intakePose[1].getX()) + intakePose[1].getY()));
  }

  /* TODO Sometimes, the auto paths ran will eject the robot out of bounds. This will reset the robot state so you can
  re-run the auto without restarting the sim*/

  public void simulationPeriodic() {

    m_field2d.setRobotPose(m_driveTrain.getRobotPoseMeters());
    m_field2d
        .getObject("Turret")
        .setPose(
            new Pose2d(
                m_driveTrain.getRobotPoseMeters().getTranslation(),
                Rotation2d.fromDegrees(getIdealAngleToHub())));

    updateIntakePoses();

    if (m_intake.getIntakePistonExtendStatus()) {
      m_field2d.getObject("Intake").setPoses(intakePose);
    } else {
      m_field2d.getObject("Intake").setPoses(intakePoseHidden);
    }

    for (Cargo p : m_redCargo) {
      updateBallState(p);
      m_field2d
          .getObject("Red Cargo")
          .setPoses(
              Arrays.stream(m_redCargo).map(Cargo::getCargoPose).collect(Collectors.toList()));
    }

    for (Cargo p : m_blueCargo) {
      updateBallState(p);
      m_field2d
          .getObject("Blue Cargo")
          .setPoses(
              Arrays.stream(m_blueCargo).map(Cargo::getCargoPose).collect(Collectors.toList()));
    }

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public double getIdealAngleToHub() {
    return Math.toDegrees(
        Math.atan2(
            SimConstants.hubPoseMeters.getY() - m_driveTrain.getRobotPoseMeters().getY(),
            SimConstants.hubPoseMeters.getX() - m_driveTrain.getRobotPoseMeters().getX()));
  }

  public Cargo[] getRedCargo() {
    return m_redCargo;
  }

  public Cargo[] getBlueCargo() {
    return m_blueCargo;
  }

  public Cargo[] getCargo() {
    cargo = new Cargo[m_redCargo.length + m_blueCargo.length];
    System.arraycopy(m_redCargo, 0, cargo, 0, m_redCargo.length);
    System.arraycopy(m_blueCargo, 0, cargo, m_redCargo.length, m_blueCargo.length);

    return cargo;
  }

  public Pose2d getRobotPose() {
    return m_field2d.getRobotPose();
  }

  /** synchronized prevents calling this method simoultaneously */
  public synchronized void resetRobotPose(Pose2d pose) {
    m_field2d.setRobotPose(pose);
    m_driveTrain.resetOdometry(pose, pose.getRotation());
  }

  private void updateBallState(Cargo cargo) {
    ballPose = cargo.getCargoPose();

    switch (cargo.getBallState()) {
      case IN_AIR:
        if (Math.pow(ballPose.getX() - SimConstants.hubPoseMeters.getX(), 2)
                + Math.pow(ballPose.getY() - SimConstants.hubPoseMeters.getY(), 2)
            < Math.pow(0.1, 2)) { // / If the ball has gone into the hub
          cargo.setBallState(BallState.ON_FIELD);
          break;
        }
        currentTime = RobotController.getFPGATime();
        // FPGA time is in microseonds, need to convert it into seconds
        deltaT = (currentTime - cargo.getLastTimestamp()) / 1e6;
        distanceTraveled = SimConstants.shotSpeedMetersPerSecond * deltaT;

        deltaX = distanceTraveled * Math.cos(Math.toRadians(getIdealAngleToHub()));
        deltaY = distanceTraveled * Math.sin(Math.toRadians(getIdealAngleToHub()));

        cargo.setBallPose(
            new Pose2d(deltaX + ballPose.getX(), deltaY + ballPose.getY(), ballPose.getRotation()));

        cargo.setLastTimestamp(currentTime);
        break;
      case IN_ROBOT:
        cargo.setBallPose(m_field2d.getObject("Turret").getPose());

        if (cargo.getBallShotState()) {
          cargo.setBallShotState(false);
          cargo.setLastTimestamp(RobotController.getFPGATime());
          cargo.setBallState(BallState.IN_AIR);
          ballCount--;
        }
        break;
      case ON_FIELD:
      default:
        if (m_intake.getIntakeState()
            && m_intake.getIntakePistonExtendStatus()
            && isBallInIntakeZone(ballPose)
            && ballCount < 2) {
          ballCount++;
          cargo.setBallState(BallState.IN_ROBOT);
        }
        break;
    }
  }

  public void periodic() {
    m_field2d.setRobotPose(m_driveTrain.getRobotPoseMeters());
    m_field2d.getObject("Turret").setPose(m_turret.getPose());
    m_field2d
        .getObject("Vision")
        .setPose(m_vision.getPoseFromHub(Constants.Vision.CAMERA_POSITION.GOAL));

    if (m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.INTAKE))
      m_field2d
          .getObject("DetectedCargo")
          .setPose(new Pose2d(m_vision.getCargoPositionFieldAbsolute(), new Rotation2d()));
    else m_field2d.getObject("DetectedCargo").setPose(new Pose2d());

    m_field2d.getObject("trajectory").setTrajectory(m_driveTrain.getCurrentTrajectory());
  }
}
