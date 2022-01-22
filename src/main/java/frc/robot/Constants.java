// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class USB {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;
  }

  public final class Climber {
    public static final int climbMotorA = 50;
    public static final int climbPistonForward = 4;
    public static final int climbPistonReverse = 8;
  }

  public final class Indexer {
    public static final int indexerMotor = 35;
    public static final int indexerTopSensor = 1;
    public static final int indexerBottomSensor = 2;
    public static final int kickerMotor = 36;
  }

  public final class Intake {
    public static final int pcmOne = 11;
    public static final int intakePistonForward = 0;
    public static final int intakePistonReverse = 1;
    public static final int intakeMotor = 47;
    public static final int intakeSensor = 0;
  }

  public static final class DriveTrain {
    public static final int[] kLeftEncoderPorts = new int[] {10, 11};
    public static final int[] kRightEncoderPorts = new int[] {12, 13};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackWidthMeters = Units.inchesToMeters(21.5);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
    public static final double kDriveGearing = 9.05; // For test bed
    // public static final double kDriveGearing = 128.0/15.0; // For Jango

    public static final int kMagEncoderCPR = 4096;
    public static final int kFalconEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = Units.feetToMeters(0.5);
    public static final double kEncoderDistancePerPulseMeters =
        // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
        (kWheelDiameterMeters * Math.PI) / (double) (kFalconEncoderCPR * kDriveGearing);
    public static final double kEncoderDistancePerPulseMetersSim =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kMagEncoderCPR * kDriveGearing;

    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 0.75514;
    public static final double kvVoltSecondsPerMeter = 2.1851;
    public static final double kaVoltSecondsSquaredPerMeter = 0.57574;

    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 3.34;
    public static final double kaVoltSecondsSquaredPerRadian = 0.19;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    public static final double kMaxVelocityMetersPerSecond = 2.0;

    public static final int leftFrontDriveMotor = 20;
    public static final int leftRearDriveMotor = 21;
    public static final int rightFrontDriveMotor = 22;
    public static final int rightRearDriveMotor = 23;

    public static enum DriveTrainNeutralMode {
      ALL_BRAKE,
      ALL_COAST,
      FOLLOWER_COAST
    }
  }

  public static final class Sim {
    public static final Pose2d[] redHubBallPos = {
      new Pose2d(Units.inchesToMeters(351), Units.inchesToMeters(13), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(168), Units.inchesToMeters(128), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(229), Units.inchesToMeters(288), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(343), Units.inchesToMeters(313), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(443), Units.inchesToMeters(251), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(447), Units.inchesToMeters(80), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(279), new Rotation2d())
    };

    public static final Pose2d[] blueHubBallPos = {
      new Pose2d(Units.inchesToMeters(478), Units.inchesToMeters(196), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(12), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(199), Units.inchesToMeters(76), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(418), Units.inchesToMeters(39), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(249), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(294), Units.inchesToMeters(313), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(55), new Rotation2d())
    };

    public static final double fieldWidthMeters = Units.feetToMeters(54);
    public static final double fieldHieghtMeters = Units.feetToMeters(27);
    public static final double ballDiameterMeters = Units.inchesToMeters(9.5);

    // TODO: update robotWidth,robotLength and intakeLength for the new robot
    public static final double robotWidthMeters = 0.673;
    public static final double robotLengthMeters = 0.838;
    public static final double intakeLengthMeters = 0.3048;
    public static final double shotSpeedMetersPerSecond = 10;

    public static final Pose2d hubPoseMeters =
        new Pose2d(
            fieldWidthMeters / 2, fieldHieghtMeters / 2, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d startPositionMeters = new Pose2d();

    public enum BallState {
      ON_FIELD,
      IN_ROBOT,
      IN_AIR,
      OUT_OF_BOUNDS
    }
  }

  public final class LED {
    public static final int ledPort = 0;
  }

  public final class Flywheel {
    public static final int flywheelMotorA = 40;
    public static final int flywheelMotorB = 41;

    public static final int encoderUnitsPerRotation = 2048;

    // Volts per (radian per second)
    public static final double kFlywheelKv = 0.017092;

    // Volts per (radian per second squared)
    public static final double kFlywheelKa = 0.0083035;

    public static final double kFlywheelKs = 0.53456;

    public static final double rpmTolerance = 25.0;
  }

  public final class Turret {
    public static final int turretMotor = 60;
    public static final int turretEncoder = 61;
    public static final int turretHomeSensor = 3;
  }

  public final class Vision {}
}
