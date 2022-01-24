package frc.robot.commands.driveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;

public class DriveBackwardDistance extends SequentialCommandGroup {
  public DriveBackwardDistance(
      DriveTrain driveTrain, FieldSim fieldSim, double distance) { // Distance in meters
    Pose2d startPosition = new Pose2d();
    Pose2d endPosition = new Pose2d(-distance, 0, new Rotation2d());
    TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(10));
    configA.setReversed(true);
    configA.setEndVelocity(0);
    configA.addConstraint(
        new DifferentialDriveKinematicsConstraint(
            driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
    configA.addConstraint(
        new DifferentialDriveVoltageConstraint(
            driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(), 10));
    ArrayList<Pose2d> driveBackwardPath = new ArrayList<Pose2d>();
    driveBackwardPath.add(startPosition);
    driveBackwardPath.add(endPosition);
    var driveBackwardCommand =
        TrajectoryUtils.generateRamseteCommand(driveTrain, driveBackwardPath, configA);

    addCommands(
        new SetOdometry(driveTrain, fieldSim, startPosition),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.FOLLOWER_COAST),
        driveBackwardCommand);
  }
}
