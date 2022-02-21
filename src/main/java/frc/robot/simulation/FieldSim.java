package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Sim.BallColor;
import frc.robot.Constants.Sim.BallState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class FieldSim {
  private final Field2d m_field2d;
  private final DriveTrain m_driveTrain;
  private final Intake m_intake;
  // Exclude 5 cargo in other robots and 2 with human players
  private final Cargo[] m_cargo = new Cargo[15];

  private int ballCount = 0;
  private final Pose2d[] intakePose = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
  private final Pose2d intakePoseHidden = new Pose2d(-50, 50, new Rotation2d());

  public FieldSim(DriveTrain driveTrain, Intake intake) {
    m_driveTrain = driveTrain;
    m_intake = intake;

    m_field2d = new Field2d();
  }

  public void initSim() {
    // Loads 1 cargo into the robot
    m_cargo[0] = new Cargo("BlueCargo_0", BallColor.BLUE, new Pose2d());
    m_cargo[0].setBallState(BallState.IN_ROBOT);

    // Places and sets the position of all the cargo on the field

    for (int i = 0; i < Constants.Sim.blueHubBallPos.length; i++) {
      m_cargo[i + 1] =
          new Cargo(
              String.format("BlueCargo_" + (i + 1)),
              BallColor.BLUE,
              Constants.Sim.blueHubBallPos[i]);
      m_cargo[i + 1].setBallState(BallState.ON_FIELD);
    }

    for (int i = 0; i < Constants.Sim.redHubBallPos.length; i++) {
      m_cargo[i + 1 + Constants.Sim.blueHubBallPos.length] =
          new Cargo(String.format("RedCargo_" + i), BallColor.RED, Constants.Sim.redHubBallPos[i]);
      m_cargo[i + 1 + Constants.Sim.blueHubBallPos.length].setBallState(BallState.ON_FIELD);
    }

    m_field2d.setRobotPose(Constants.Sim.startPositionMeters);
    m_driveTrain.resetOdometry(
        Constants.Sim.startPositionMeters, Constants.Sim.startPositionMeters.getRotation());
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
    Pose2d robotPose = m_driveTrain.getRobotPoseMeters();
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double cos = robotPose.getRotation().getCos();
    double sin = robotPose.getRotation().getSin();

    double deltaXa =
        (robotX - Constants.Sim.robotLengthMeters / 2.0 - Constants.Sim.intakeLengthMeters)
            - robotX;
    double deltaXb = (robotX - Constants.Sim.robotLengthMeters / 2.0) - robotX;
    double deltaYa = (robotY + Constants.Sim.robotWidthMeters / 2.0) - robotY;
    double deltaYb = (robotY - Constants.Sim.robotWidthMeters / 2.0) - robotY;

    intakePose[0] =
        new Pose2d(
            cos * deltaXa - sin * deltaYa + robotX,
            sin * deltaXa + cos * deltaYa + robotY,
            new Rotation2d());
    intakePose[1] =
        new Pose2d(
            cos * deltaXa - sin * deltaYb + robotX,
            sin * deltaXa + cos * deltaYb + robotY,
            new Rotation2d());
    intakePose[2] =
        new Pose2d(
            cos * deltaXb - sin * deltaYb + robotX,
            sin * deltaXb + cos * deltaYb + robotY,
            new Rotation2d());
    intakePose[3] =
        new Pose2d(
            cos * deltaXb - sin * deltaYa + robotX,
            sin * deltaXb + cos * deltaYa + robotY,
            new Rotation2d());
  }

  private boolean isBallInIntakeZone(Pose2d ballPose) {
    // The rise/run between intake points 0 to 1
    // Since the intake is a rectangle, this is the same as the slope between points 2 to 3
    double slope0to1 =
        (intakePose[1].getY() - intakePose[0].getY())
            / (intakePose[1].getX() - intakePose[0].getX());

    // The rise/run between points 1 to 2
    // Same as slope between points 3 and 0
    double slope1to2 =
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
      m_field2d.getObject("Intake A").setPose(intakePose[0]);
      m_field2d.getObject("Intake B").setPose(intakePose[1]);
      m_field2d.getObject("Intake C").setPose(intakePose[2]);
      m_field2d.getObject("Intake D").setPose(intakePose[3]);
    } else {
      m_field2d.getObject("Intake A").setPose(intakePoseHidden);
      m_field2d.getObject("Intake B").setPose(intakePoseHidden);
      m_field2d.getObject("Intake C").setPose(intakePoseHidden);
      m_field2d.getObject("Intake D").setPose(intakePoseHidden);
    }

    for (Cargo p : m_cargo) {
      updateBallState(p);
      m_field2d.getObject(p.getName()).setPose(p.getBallPose());
    }

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public double getIdealAngleToHub() {
    return Math.toDegrees(
        Math.atan2(
            Constants.Sim.hubPoseMeters.getY() - m_driveTrain.getRobotPoseMeters().getY(),
            Constants.Sim.hubPoseMeters.getX() - m_driveTrain.getRobotPoseMeters().getX()));
  }

  public Cargo[] getCargo() {
    return m_cargo;
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
    Pose2d ballPose = cargo.getBallPose();

    switch (cargo.getBallState()) {
      case IN_AIR:
        if (Math.pow(ballPose.getX() - Constants.Sim.hubPoseMeters.getX(), 2)
                + Math.pow(ballPose.getY() - Constants.Sim.hubPoseMeters.getY(), 2)
            < Math.pow(0.1, 2)) { // / If the ball has gone into the hub
          cargo.setBallState(BallState.ON_FIELD);
          break;
        }
        double currentTime = RobotController.getFPGATime();
        // FPGA time is in microseonds, need to convert it into seconds
        double deltaT = (currentTime - cargo.getLastTimestamp()) / 1e6;
        double distanceTraveled = Constants.Sim.shotSpeedMetersPerSecond * deltaT;

        double deltaX = distanceTraveled * Math.cos(Math.toRadians(getIdealAngleToHub()));
        double deltaY = distanceTraveled * Math.sin(Math.toRadians(getIdealAngleToHub()));

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

  public static class Cargo {
    private BallColor m_color;
    private boolean wasShot;
    private Pose2d ballPose = new Pose2d();
    private double m_lastTimestamp;
    private BallState ballState = BallState.ON_FIELD;

    String m_name;

    public Cargo(String name, BallColor color, Pose2d pose) {
      m_name = name;
      m_color = color;
      ballPose = pose;
    }

    public String getName() {
      return m_name;
    }

    public BallState getBallState() {
      return ballState;
    }

    public boolean getBallShotState() {
      return wasShot;
    }

    public void setBallState(BallState state) {
      ballState = state;
    }

    public void setBallShotState(boolean shotState) {
      wasShot = shotState;
    }

    public void setBallPose(Pose2d pose) {
      ballPose = pose;
    }

    public Pose2d getBallPose() {
      return ballPose;
    }

    public void setLastTimestamp(double timestamp) {
      m_lastTimestamp = timestamp;
    }

    public double getLastTimestamp() {
      return m_lastTimestamp;
    }

    public BallColor getColor() {
      return m_color;
    }
  }
}
