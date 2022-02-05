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

  public static final class Pneumatics {
    public static final int pcmOne = 11;
    public static final int intakePistonForward = 0;
    public static final int intakePistonReverse = 1;
    public static final int climbPistonForward = 2;
    public static final int climbPistonReverse = 3;
  }

  public final class Climber {
    public static final int climbMotorA = 50;
    public static final int climbMotorB = 51;

    public static final double climberBottomOutValue = 0;
    public static final double climberTopOutValue = 1;
  }

  public static final class Indexer {
    public static final int indexerMotor = 35;
    public static final int kickerMotor = 36;
    public static final int indexerRearSensor = 1;
    public static final int indexerFrontSensor = 2;
    public static final int colorSensorFront = 2;
    public static final int colorSensorRear = 0;

    public static enum CARGO_COLOR {
      RED,
      BLUE,
      UNKNOWN
    }
  }

  public final class Intake {
    public static final int intakeMotor = 30;
    // public static final int intakeSensor = 0;
  }

  public static final class DriveTrain {
    // Number identification for the CAN motors
    public static final int leftFrontDriveMotor = 20;
    public static final int leftRearDriveMotor = 21;
    public static final int rightFrontDriveMotor = 22;
    public static final int rightRearDriveMotor = 23;

    public enum MotorPosition {
      LEFT_FRONT,
      LEFT_REAR,
      RIGHT_FRONT,
      RIGHT_REAR
    }

    public enum DriveTrainNeutralMode {
      BRAKE,
      COAST,
      /** Master motors brake, follower motors coast */
      HALF_BRAKE
    }

    public static final double kTrackWidthMeters = Units.inchesToMeters(21.5);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxVelocityMetersPerSecond = 2.0;
    public static final double kDriveGearing = 9.05;
    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);

    public static final int kCANcoderCPR = 4096;
    public static final int kFalconEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = Units.feetToMeters(0.5);
    public static final double kEncoderDistancePerPulseMeters =
        // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
        (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveGearing);

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
  }

  public static final class Sim {
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
      IN_AIR
    }

    public enum BallColor {
      RED,
      BLUE
    }
  }

  public final class LED {
    public static final int CANdleID = 0;
  }

  public final class Flywheel {
    public static final int flywheelMotorA = 40;
    public static final int flywheelMotorB = 41;

    public static final int encoderUnitsPerRotation = 2048;

    public static final double kFlywheelKs = 0.53456; // 0.63348; // Jamgo: 0.53456;
    public static final double kFlywheelKv = 0.017092; // 0.01;//0.15784; // Jamgo: 0.017092;
    public static final double kFlywheelKa = 0.0083035; // 0.008;//0.034438; // Jamgo: 0.0083035;

    public static final double rpmTolerance = 25.0;

    public static final double gearRatio = 1.0; // 2.0 / 3.0;
  }

  public final class Turret {
    public static final int turretMotor = 60;
    public static final int turretEncoder = 61;

    // TODO: might not need kErrorBand, need to confirm
    public static final int kErrorBand = 50;
    public static final int turretHomeSensor = 3;

    public static final double kTurretKs = 1.3661;
    public static final double kTurretKv = 0.068821;
    public static final double kTurretKa = 0.0063138;

    public static final double degreeTolerance = 1.0;

    public static final double canCodertoTurretGearRatio = 120.0 / 18.0;

    public static final double toTurretGearRatio = 27.0 / 1.0;
  }

  public static final class Vision {
    public enum CAMERA_TYPE {
      OAK_D,
      LIMELIGHT,
      PHOTONVISION
    }

    public static double CAMERA_MOUNTING_ANGLE_DEGREES = 30.0;

    /*
     * Co-Processor IP Addresses
     * 10.42.1.100: Goal Camera
     * 10.42.1.101: Intake Camera
     */
    public static String goalCameraIP = "10.42.1.100";
    public static String intakeCameraIP = "10.42.1.101";
  }
}
