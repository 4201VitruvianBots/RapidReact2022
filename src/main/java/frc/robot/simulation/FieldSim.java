package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Sim.BallState;
import frc.robot.subsystems.DriveTrain;

public class FieldSim {
  private final Field2d m_field2d;
  private final DriveTrain m_driveTrain;
  private final Cargo[] m_cargo = new Cargo[15]; // Do not include the 5 cargo that will be in other robots or 2 with the human players

  private int ballCount;
  private final Pose2d[] intakePose = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};

  private double m_autoStartTime;

  public FieldSim(DriveTrain driveTrain /*, Outtake outtake*/) {
    m_driveTrain = driveTrain;

    for (int i = 0; i < m_cargo.length; i++)
      m_cargo[i] = new Cargo(String.format("Cargo_" + String.format("%02d", i)));

    m_field2d = new Field2d();
  }

  public void initSim() {
    // Loads 1 cargo into the robot
    m_cargo[0].setBallState(BallState.IN_ROBOT);

    // Places all other cargo onto the field
    for (int i = 1; i < m_cargo.length; i++) m_cargo[i].setBallState(BallState.ON_FIELD);

    // Sets the position of all the cargo on the field
    m_cargo[1].setBallPose(Constants.Sim.blueHubBallPos[0]);
    m_cargo[2].setBallPose(Constants.Sim.blueHubBallPos[1]);
    m_cargo[3].setBallPose(Constants.Sim.blueHubBallPos[2]);
    m_cargo[4].setBallPose(Constants.Sim.blueHubBallPos[3]);
    m_cargo[5].setBallPose(Constants.Sim.blueHubBallPos[4]);
    m_cargo[6].setBallPose(Constants.Sim.blueHubBallPos[5]);
    m_cargo[7].setBallPose(Constants.Sim.blueHubBallPos[6]);

    m_cargo[8].setBallPose(Constants.Sim.redHubBallPos[0]);
    m_cargo[9].setBallPose(Constants.Sim.redHubBallPos[1]);
    m_cargo[10].setBallPose(Constants.Sim.redHubBallPos[2]);
    m_cargo[11].setBallPose(Constants.Sim.redHubBallPos[3]);
    m_cargo[12].setBallPose(Constants.Sim.redHubBallPos[4]);
    m_cargo[13].setBallPose(Constants.Sim.redHubBallPos[5]);
    m_cargo[14].setBallPose(Constants.Sim.redHubBallPos[6]);

    m_field2d.setRobotPose(Constants.Sim.startPositionMeters);
    m_driveTrain.resetOdometry(
        Constants.Sim.startPositionMeters, Constants.Sim.startPositionMeters.getRotation());
    m_autoStartTime = Timer.getFPGATimestamp();
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

  /*  Sometimes, the auto paths ran will eject the robot out of bounds. This will reset the robot state so you can
     re-run the auto without restarting the sim
  */

  public void simulationPeriodic() {

    m_field2d.setRobotPose(m_driveTrain.getRobotPoseMeters());
    m_field2d.getObject("Outtake").setPose(m_driveTrain.getRobotPoseMeters());

    updateIntakePoses();

    m_field2d.getObject("Intake A").setPose(intakePose[0]);
    m_field2d.getObject("Intake B").setPose(intakePose[1]);
    m_field2d.getObject("Intake C").setPose(intakePose[2]);
    m_field2d.getObject("Intake D").setPose(intakePose[3]);

    for (Cargo p : m_cargo) {
      updateBallState(p);
      m_field2d.getObject(p.getName()).setPose(p.getBallPose());
    }

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public double getAutoStartTime() {
    return m_autoStartTime;
  }

  public Cargo[] getCargo() {
    return m_cargo;
  }

  public Pose2d getRobotPose() {
    return m_field2d.getRobotPose();
  }

  public synchronized void resetRobotPose(Pose2d pose) {
    m_field2d.setRobotPose(pose);
    m_driveTrain.resetOdometry(pose, pose.getRotation());
  }

  private void updateBallState(Cargo cargo) {
    Pose2d ballPose = cargo.getBallPose();
    if (cargo.getBallState() != BallState.IN_ROBOT) {
      if (ballPose.getX() < 0
          || ballPose.getX() > Constants.Sim.fieldWidthMeters
          || ballPose.getY() < 0
          || ballPose.getY() > Constants.Sim.fieldHieghtMeters)
        cargo.setBallState(BallState.OUT_OF_BOUNDS);
    }

    switch (cargo.getBallState()) {
      case OUT_OF_BOUNDS:
        cargo.setBallState(BallState.ON_FIELD);
        break;
      case IN_AIR:
        // Ball is traveling in the air
        double currentTime = RobotController.getFPGATime();
        // FPGA time is in microseonds, need to convert it into seconds
        double deltaT = (currentTime - cargo.getLastTimestamp()) / 1e6;
        // double distanceTraveled = Constants.Sim.shotSpeed * deltaT;

        double deltaX = cargo.getBallVel().getX() * deltaT;
        double deltaY = cargo.getBallVel().getY() * deltaT;

        cargo.setBallPose(
            new Pose2d(deltaX + ballPose.getX(), deltaY + ballPose.getY(), ballPose.getRotation()));

        cargo.setLastTimestamp(currentTime);
        break;
      case IN_ROBOT:
        // Ball has been picked up by the robot
        cargo.setBallPose(m_field2d.getObject("Outtake").getPose());

        // Ball has been shot;
        if (cargo.getBallShotState()) {
          cargo.setBallShotState(false);
          cargo.setLastTimestamp(RobotController.getFPGATime());
          cargo.setBallState(BallState.IN_AIR);
          ballCount--;
        }
        break;
      case ON_FIELD:
      default:
        if (isBallInIntakeZone(ballPose) && ballCount < 2) {
          ballCount++;
          cargo.setBallState(BallState.IN_ROBOT);
        }
        break;
    }
  }
}
