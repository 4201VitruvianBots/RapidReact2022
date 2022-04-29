package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.intake.ReverseIntakeIndexer;
import frc.robot.commands.simulation.SetSimTrajectory;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Intakes one cargo and shoots two cargo into the high goal. */
public class ReplacementAuto extends SequentialCommandGroup {
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
  public ReplacementAuto(
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

    // shoot 1
    // drive backwards while intaking
    // shoot (at low rpm - how low?) towards [hangar? driverstation? terminal? --> hoarding problem]

    Trajectory trajectory1 =
        PathPlanner.loadPath(
            "ReplacementAutoMaster", Units.feetToMeters(6), Units.feetToMeters(3), false);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    // USE THIS? Command cargoVisionCommand = new CargoTrajectoryRameseteCommand(driveTrain,
    // vision);
    /**
     * Order of operations: drivetrain & intake & indexer & vision run until drivetrain stops
     * (except for vision) run indexer & flywheel until indexer stops end sequence Turn and move
     * forward to line up with blue ball on other side of the line (NOT running intake, indexer,
     * shooter or vision) End path
     */
    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1),
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        new ReverseIntakeIndexer(intake, indexer, -0.7).withTimeout(2),
        command1);
  }
}
