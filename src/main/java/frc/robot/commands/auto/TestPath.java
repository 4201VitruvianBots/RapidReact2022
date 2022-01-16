package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.BrakeMode;
import frc.robot.commands.driveTrain.SetDriveNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Intakes one cargo and shoots two cargo into the high goal. */
public class TestPath extends SequentialCommandGroup {
    /**
     * Intakes one cargo and shoots two cargo into the high goal.
     *
     * @param driveTrain The driveTrain used by this command.
     * @param fieldSim The fieldSim used by this command.
     */
    public TestPath(DriveTrain driveTrain, FieldSim fieldSim) {
        // Drive backward maximum distance to ball
        // While dirivng backward, intake is running
        // Stop (now with 2 cargo) and aim for high goal
        // Shoot 2 cargo into high goal

        Trajectory trajectory = PathPlanner.loadPath("Testpath", Units.feetToMeters(4), Units.feetToMeters(4), false);

        VitruvianRamseteCommand command = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);
        
        addCommands(
            new SetOdometry(driveTrain, fieldSim, trajectory.getInitialPose()),
            new SetDriveNeutralMode(driveTrain, BrakeMode.FOLLOWER_COAST),
            command,
            new SetDriveNeutralMode(driveTrain, BrakeMode.ALL_BRAKE));
        
    }
}
