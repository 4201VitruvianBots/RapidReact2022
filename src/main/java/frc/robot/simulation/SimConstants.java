package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SimConstants {
  public static final double robotWidthMeters = Units.inchesToMeters(26.5);
  public static final double robotLengthMeters = Units.inchesToMeters(33);
  public static final double intakeLengthMeters = 0.3048;

  public static final Pose2d[] redHubBallPos = {
    new Pose2d(Units.inchesToMeters(362), Units.inchesToMeters(13), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(178), Units.inchesToMeters(128), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(239), Units.inchesToMeters(288), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(354), Units.inchesToMeters(313), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(453), Units.inchesToMeters(251), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(457), Units.inchesToMeters(80), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(612), Units.inchesToMeters(280), new Rotation2d())
  };

  public static final Pose2d[] blueHubBallPos = {
    new Pose2d(Units.inchesToMeters(478), Units.inchesToMeters(196), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(12), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(199), Units.inchesToMeters(76), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(418), Units.inchesToMeters(39), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(249), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(294), Units.inchesToMeters(313), new Rotation2d()),
    new Pose2d(Units.inchesToMeters(44), Units.inchesToMeters(42), new Rotation2d())
  };

  public static final double fieldWidthMeters = Units.feetToMeters(54);
  public static final double fieldHieghtMeters = Units.feetToMeters(27);
  public static final double ballDiameterMeters = Units.inchesToMeters(9.5);

  public static final double shotSpeedMetersPerSecond = 10;

  public static final Pose2d hubPoseMeters =
      new Pose2d(
          fieldWidthMeters / 2, fieldHieghtMeters / 2, new Rotation2d(Units.degreesToRadians(0)));
  public static final Pose2d startPositionMeters = new Pose2d();

  public enum BallState {
    ON_FIELD,
    IN_ROBOT,
    IN_AIR
  }

  public enum BallColor {
    RED,
    BLUE
  }
}
