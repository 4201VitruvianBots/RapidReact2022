package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.List;

/** Drives off the tarmac and shoots two cargo into the high goal. */
public class OneBallAuto extends SequentialCommandGroup {
  /**
   * Drives off the tarmac and shoots two cargo into the high goal.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   */
  public OneBallAuto(
      DriveTrain driveTrain,
      FieldSim fieldSim,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {
    // Drive backward
    // Stop and aim for high goal
    // Shoot 1 cargo into high goal
    Trajectory trajectory1 =
        PathPlanner.loadPath("OneBallAuto-1", Units.feetToMeters(2), Units.feetToMeters(2), true);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath("OneBallAuto-2", Units.feetToMeters(2), Units.feetToMeters(2), true);

    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Pose2d[] waypoints = new Pose2d[waypointsRaw.length];
    for (int j = 0; j < waypointsRaw.length; j++) {
      waypoints[j] =
          new Pose2d(
              Units.inchesToMeters(waypointsRaw[j][0]),
              Units.inchesToMeters(waypointsRaw[j][1]),
              new Rotation2d(Units.degreesToRadians(waypointsRaw[j][2])));
    }

    Pose2d startPosition = waypoints[0]; // starting point is the first set in the waypoints array

    TrajectoryConfig configA =
        new TrajectoryConfig(Units.feetToMeters(10), Units.feetToMeters(6)); // 10 -
    // acceleration,
    // 6 -
    // speed
    // (m/s
    // and
    // m/s/s)
    configA.setReversed(true); // 'true' would make it go the opposite direction
    // configA.setEndVelocity(configA.getMaxVelocity());
    configA.addConstraint(
        new DifferentialDriveKinematicsConstraint(
            driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
    configA.addConstraint(
        new DifferentialDriveVoltageConstraint(
            driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(), 10));
    configA.addConstraint(
        new CentripetalAccelerationConstraint(1.7)); // This is what we can change when
    // we're actually testing (turning
    // speed)

    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.HALF_BRAKE),
        new SetAndHoldRpmSetpoint(flywheel, vision, 3000),
        command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new FeedAll(indexer),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal)
        // command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0))
        // Rev up flywheel while driving backwards
        // Once finish driving, feed indexer
        // After that, stop indexer and flywheel
        );
  }
}
