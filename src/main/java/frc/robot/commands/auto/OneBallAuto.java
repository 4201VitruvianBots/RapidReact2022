package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetRpmSetpoint;
import frc.robot.commands.indexer.FeedAll;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Drives off the tarmac and shoots two cargo into the high goal. */
public class OneBallAuto extends SequentialCommandGroup {
  /**
   * Drives off the tarmac and shoots two cargo into the high goal.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim   The fieldSim used by this command.
   * @param indexer    To feed the cargo to the shooter
   * @param flywheel   To shoot the cargo
   * @param vision     To find the target
   */
  public OneBallAuto(DriveTrain driveTrain, FieldSim fieldSim, Indexer indexer, Flywheel flywheel, Vision vision) {
    // Drive backward maximum distance to ball
    // Stop and aim for high goal
    // Shoot 1 cargo into high goal
    Trajectory trajectory =
      PathPlanner.loadPath("OneBallAuto", Units.feetToMeters(2), Units.feetToMeters(2), true);

    VitruvianRamseteCommand command =
      TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);

    addCommands(
      new SetOdometry(driveTrain, fieldSim, trajectory.getInitialPose()),
      new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.FOLLOWER_COAST),
      // Rev up flywheel while driving backwards
      // Once finish driving, feed indexer
      // After that, stop indexer and flywheel
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          command,
          new FeedAll(indexer)
        ),
        new SetRpmSetpoint(flywheel, vision, 1000)
        // TODO: Add vision aiming
      )

    );
  }
}
