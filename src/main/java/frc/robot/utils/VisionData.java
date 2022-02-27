package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionData {
  public double timestamp;
  public double tx;
  public double ty;
  public Pose2d robotPose;
  public Pose2d targetPose;

  public VisionData(double timestamp, double tx, double ty, Pose2d robotPose) {
    this.timestamp = timestamp;
    this.tx = tx;
    this.ty = ty;
    this.robotPose = robotPose;
  }
}
