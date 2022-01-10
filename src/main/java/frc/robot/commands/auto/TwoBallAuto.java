package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
// import frc.robot.commands.auto.SmartdashboardCommand;
// import frc.robot.commands.auto.TurnInPlace;
import frc.robot.commands.driveTrain.SetDriveNeutralMode;
//import frc.robot.commands.driveTrain.SetDriveShifters;
import frc.robot.commands.driveTrain.SetOdometry;
// import frc.robot.commands.intake.SetIntakeSpeed;
// import frc.robot.commands.intake.SetIntakePiston;
// import frc.robot.commands.intake.SetIntakeStates;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;

/** intakes one cargo and intakes two cargo into the high goal */
public class TwoBallAuto extends SequentialCommandGroup {
    /**
     * Creates a new TwoBallAuto.
     *
     * @param driveTrain The driveTrain used by this command.
     */
    public TwoBallAuto(DriveTrain driveTrain, FieldSim fieldSim) {
        // Drive backward maximum distance to ball
        // While dirivng backward, intake is running
        // Stop (now with 2 cargo) and aim for high goal
        // Shoot 2 cargo into high goal

        int[][] waypointsRaw = { // x, y, and z coordinates of the robot (x and y in inches, z is degrees of
                                 // rotation)
                { 0, 0, 0 },
                { -120, 0, 0 },

        };

        /*
         * Runs through each of the set of x, y and z values in the list below one at a
         * time.
         * At each set (starting at 0), it merges the x and y values of that set into on
         * Pose2d coordinate
         * The varibale j is used to represent which set in the list is currently
         * processing (starting at 0)
         * The [0] and [1] specify the values being merged are from the x [0] and y [1]
         * columns of the list.
         */

        Pose2d[] waypoints = new Pose2d[waypointsRaw.length];
        for (int j = 0; j < waypointsRaw.length; j++) {
            waypoints[j] = new Pose2d(Units.inchesToMeters(waypointsRaw[j][0]),
                    Units.inchesToMeters(waypointsRaw[j][1]),
                    new Rotation2d(Units.degreesToRadians(waypointsRaw[j][2])));
        }

        Pose2d startPosition = waypoints[0]; // starting point is the first set in the waypoints array

        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(6)); // 10 -
                                                                                                        // acceleration,
                                                                                                        // 6 -
                                                                                                        // speed
                                                                                                        // (m/s
                                                                                                        // and
                                                                                                        // m/s/s)
        configA.setReversed(false); // 'true' would make it go the opposite direction
        // configA.setEndVelocity(configA.getMaxVelocity());
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(),
                configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(),
                driveTrain.getDriveTrainKinematics(), 10));
        configA.addConstraint(new CentripetalAccelerationConstraint(1.7)); // This is what we can change when
                                                                           // we're actually testing (turning
                                                                           // speed)

        addCommands(
                new SetOdometry(driveTrain, fieldSim, startPosition),
                new SetDriveNeutralMode(driveTrain, 0));

        double[] startVelocities = {
                0, // starting velocity from point 0 to point 1
                2 * configA.getMaxVelocity() / 3, // starting velocity from point 1 to point 2
                2 * configA.getMaxVelocity() / 3,
                2 * configA.getMaxVelocity() / 3,
                2 * configA.getMaxVelocity() / 3,
                2 * configA.getMaxVelocity() / 3,
                2 * configA.getMaxVelocity() / 3,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4 };
        double[] endVelocities = {
                2 * configA.getMaxVelocity() / 3, // ending velocity at point 1 from point 0
                2 * configA.getMaxVelocity() / 3, // ending velocity at point 2 from point 1
                2 * configA.getMaxVelocity() / 3,
                2 * configA.getMaxVelocity() / 3,
                2 * configA.getMaxVelocity() / 3,
                2 * configA.getMaxVelocity() / 3,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4,
                3 * configA.getMaxVelocity() / 4,
                2 * configA.getMaxVelocity() / 3 };

        for (int i = 0; i < waypoints.length - 1; i++) {
            configA.setStartVelocity(startVelocities[i]);
            configA.setEndVelocity(endVelocities[i]);

            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                    List.of(),
                    waypoints[i + 1], // makes a path from one waypoint to the next
                    configA); // makes sure that path fits under the constraints
            var command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
            addCommands(command);
        }
    }
}
