package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Intakes one cargo and shoots two cargo into the high goal. */
public class TwoBallAuto extends SequentialCommandGroup {
  /**
   * Intakes one cargo and shoots two cargo into the high goal.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   * @param intake Runs the intake to pick up new cargo.
   * @param indexer Feeds cargo through indexer to shoot.
   * @param flywheel Rev flywheel to shoot.
   * @param turret Turn turret to goal.
   * @param vision Find target.
   */
  public TwoBallAuto(
      DriveTrain driveTrain,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {
    // Drive backward maximum distance to ball
    // While dirivng backward, intake is running
    // Stop (now with 2 cargo) and aim for high goal
    // Shoot 2 cargo into high goal

    Trajectory trajectory =
        PathPlanner.loadPath("TwoBallAuto", Units.feetToMeters(4), Units.feetToMeters(4), true);

    VitruvianRamseteCommand command =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);

    Trajectory trajectory2 =
        PathPlanner.loadPath("TwoBallAuto-2", Units.feetToMeters(4), Units.feetToMeters(4), true);

    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    /**
     * Order of operations: drivetrain & intake & indexer & vision run until drivetrain stops
     * (except for vision) run indexer & flywheel until indexer stops end sequence
     * Turn and move forward to line up with blue ball on other side of the line (NOT running intake, indexer, shooter or vision)
     * End path
     */
    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.HALF_BRAKE),
        new SetIntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 3000),
        new ParallelDeadlineGroup(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoControlledIntake(intake, indexer)
            // TODO implement indexer
            ),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        // TODO how long does flywheel take to rev up? (should the flywheel run while
        // driving?)
        new SetIntakePiston(intake, false),
        new ConditionalCommand(
            new FeedAll(indexer),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)));
  }
}
