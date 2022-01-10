package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SimConstants {
    public static final double fieldWidth = 15.980;
    public static final double fieldHieght = 8.210;
    public static final double ballDiameter = 0.1778; // In meters

    public static final double robotWidth = 0.673;
    public static final double robotLength = 0.838;
    public static final double intakeLength = 0.3048;
    public static final double shotSpeed = 10; // in meters/second;

    public static final Pose2d redLoadingStation = new Pose2d(0.238258, 2.554548, new Rotation2d());
    public static final Pose2d blueLoadingStation = new Pose2d(15.732665, 5.646024, new Rotation2d());

    public static final Pose2d[] blueTrenchBallPos = {
            new Pose2d(6.154554,7.506032, new Rotation2d()),
            new Pose2d(7.064754,7.506032, new Rotation2d()),
            new Pose2d(7.993157,7.506032, new Rotation2d()),
            new Pose2d(9.615867,7.275692, new Rotation2d()),
            new Pose2d(9.615867,7.744320, new Rotation2d())
    };

    public static final Pose2d[] blueCenterBalls = {
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(6.340368,5.299097, new Rotation2d()),
            new Pose2d(5.943837,5.118855, new Rotation2d()),
    };

    public static final Pose2d[] redTrenchBallPos = {
            new Pose2d(9.814133,0.660829, new Rotation2d()),
            new Pose2d(8.894900,0.660829, new Rotation2d()),
            new Pose2d(7.975669,0.660829, new Rotation2d()),
            new Pose2d(6.340368,0.919229, new Rotation2d()),
            new Pose2d(6.340368,0.432577, new Rotation2d()),
    };

    public static final Pose2d[] redCenterBalls = {
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(10.012398,3.040016, new Rotation2d()),
            new Pose2d(9.615867,2.877799, new Rotation2d()),
    };

    public static final Pose2d blueGoalPose = new Pose2d(0, 5.831, new Rotation2d());
    public static final Pose2d redGoalPose = new Pose2d(15.960367,2.373122, new Rotation2d());

//    int numberOfCones = SimConstants.autoNavSlalomCones.length;
//        for(int i = 0; i < numberOfCones; i++) {
//        var constraint = new EllipticalRegionConstraint(
//                SimConstants.autoNavSlalomCones[i],
//                Units.inchesToMeters(30),
//                Units.inchesToMeters(30),
//                new Rotation2d(),
//                new SwerveDriveKinematicsConstraint(Constants.DriveConstants.kDriveKinematics, 0)
//        );
//        config.addConstraint(constraint);
//    }

    public static final Translation2d[] autoNavSlalomCones = {
            new Translation2d(3.088189,1.513965),
            new Translation2d(3.818932,1.513965),
            new Translation2d(4.549675,1.513965),
            new Translation2d(5.313633,1.513965),
            new Translation2d(6.044377,1.513965),
            new Translation2d(7.550150,1.513965)
    };

    public static final Pose2d[] GalacticSearchARedBalls = {
        new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(90), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(60), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d()),
    };
    
    public static final Pose2d[] GalacticSearchABlueBalls = {
        new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(30), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(90), new Rotation2d()),
    };

    public static final Pose2d[] GalacticSearchBRedBalls = {
        new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(120), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(60), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(120), new Rotation2d())
    };
    
    public static final Pose2d[] GalacticSearchBBlueBalls = {
        new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(60), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(120), new Rotation2d()),
        new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(60), new Rotation2d())
    };
}
