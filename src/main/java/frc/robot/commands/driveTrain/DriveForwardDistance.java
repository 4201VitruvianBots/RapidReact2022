package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveTrain.SetDriveNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public class DriveForwardDistance extends SequentialCommandGroup {
    public DriveForwardDistance(DriveTrain driveTrain, FieldSim fieldSim, double distance) { // Distance in meters
        Pose2d startPosition = new Pose2d();
        Pose2d endPosition = new Pose2d(distance, 0, new Rotation2d());
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(1));
        configA.setReversed(false);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(),10));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPosition,
        List.of(),
        endPosition,
        configA);
        
        var driveForwardCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);

        addCommands(
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain,0),
                driveForwardCommand
            );
    }
}

