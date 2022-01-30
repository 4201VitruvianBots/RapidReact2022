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

/** Runs the TestPath trajectory. */
public class TestPath extends SequentialCommandGroup {
  /**
   * Runs the TestPath trajectory.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   */
  public TestPath(DriveTrain driveTrain, FieldSim fieldSim) {

    Trajectory trajectory =
        PathPlanner.loadPath("Testpath", Units.feetToMeters(1), Units.feetToMeters(1), true);

    VitruvianRamseteCommand command =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);

    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.HALF_BRAKE),
        command,
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE));
  }
}
