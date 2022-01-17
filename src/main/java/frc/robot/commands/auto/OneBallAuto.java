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

/** Drives off the tarmac and shoots two cargo into the high goal. */
public class OneBallAuto extends SequentialCommandGroup {
  /**
   * Drives off the tarmac and shoots two cargo into the high goal.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   */
  public OneBallAuto(DriveTrain driveTrain, FieldSim fieldSim) {
    // Drive backward maximum distance to ball
    // Stop and aim for high goal
    // Shoot 1 cargo into high goal
    Trajectory trajectory =
        PathPlanner.loadPath("OneBallAuto", Units.feetToMeters(4), Units.feetToMeters(4), true);

    VitruvianRamseteCommand command =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);

    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.FOLLOWER_COAST),
        command);
  }
}
