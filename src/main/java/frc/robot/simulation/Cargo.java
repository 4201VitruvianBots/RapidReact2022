package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;

public class Cargo {
  String m_name;
  SimConstants.BallColor m_color;
  Pose2d m_pose = new Pose2d();
  private boolean wasShot;
  private double m_lastTimestamp;
  private SimConstants.BallState ballState = SimConstants.BallState.ON_FIELD;

  public Cargo(String name, SimConstants.BallColor color, Pose2d pose) {
    m_name = name;
    m_color = color;
    m_pose = pose;
  }

  public String getName() {
    return m_name;
  }

  public SimConstants.BallState getBallState() {
    return ballState;
  }

  public boolean getBallShotState() {
    return wasShot;
  }

  public void setBallState(SimConstants.BallState state) {
    ballState = state;
  }

  public void setBallShotState(boolean shotState) {
    wasShot = shotState;
  }

  public void setBallPose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getCargoPose() {
    return m_pose;
  }

  public void setLastTimestamp(double timestamp) {
    m_lastTimestamp = timestamp;
  }

  public double getLastTimestamp() {
    return m_lastTimestamp;
  }

  public SimConstants.BallColor getColor() {
    return m_color;
  }
}
