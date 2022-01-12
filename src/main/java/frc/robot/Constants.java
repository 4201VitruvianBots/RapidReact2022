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

    }

    public final static class DriveTrain {
        public static final int[] kLeftEncoderPorts = new int[]{10, 11};
        public static final int[] kRightEncoderPorts = new int[]{12, 13};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackWidthMeters = Units.inchesToMeters(21.5);
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
        public static final double kDriveGearing = 5.0;

        public static final int kMagEncoderCPR = 4096;
        public static final int kFalconEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = Units.feetToMeters(0.5);
        public static final double kEncoderDistancePerPulseMeters =
                // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
                (kWheelDiameterMeters * Math.PI) / (double) kFalconEncoderCPR * kDriveGearing;
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
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

        public static final double kMaxVelocityMetersPerSecond = 2.0;

        public static final int leftFrontDriveMotor = 20;
        public static final int leftRearDriveMotor = 21;
        public static final int rightFrontDriveMotor = 22;
        public static final int rightRearDriveMotor = 23;
    }

    public static final class Sim {
        public static final double fieldWidthMeters = Units.feetToMeters(54);
        public static final double fieldHieghtMeters = Units.feetToMeters(27);
        public static final double ballDiameterMeters = Units.inchesToMeters(9.5);
    
        public static final double robotWidth = 0.673;
        public static final double robotLength = 0.838;
        public static final double intakeLength = 0.3048;
        public static final double shotSpeed = 10; // in meters/second;
    
        public static final Pose2d redLoadingStation = new Pose2d(0.238258, 2.554548, new Rotation2d());
        public static final Pose2d blueLoadingStation = new Pose2d(15.732665, 5.646024, new Rotation2d());
    
        public static final Pose2d blueGoalPose = new Pose2d(0, 5.831, new Rotation2d());
        public static final Pose2d redGoalPose = new Pose2d(15.960367,2.373122, new Rotation2d());
    }

    public final class Indexer {

    }

    public final class Intake {

    }

    public final class LED {

    }

    public final class Outtake {

    }

    public final class Vision {

    }

}
