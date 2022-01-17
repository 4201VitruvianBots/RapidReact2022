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

/** Scores 3 cargo in the high goal and exits the tarmac */
public class ThreeBallAuto extends SequentialCommandGroup {
    /**
     * Scores 3 cargo in the high goal and exits the tarmac.
     *
     * @param driveTrain The driveTrain used by this command.
     * @param fieldSim The fieldSim used by this command.
     */
    public ThreeBallAuto(DriveTrain driveTrain, FieldSim fieldSim) {
        // Drive backward maximum distance to ball
        // While dirivng backward, intake is running
        // Stop (now with 2 cargo) and aim for high goal
        // Shoot 2 cargo into high goal


        Trajectory trajectory1 = PathPlanner.loadPath("ThreeBallAuto-1", Units.feetToMeters(10), Units.feetToMeters(6), true);
        VitruvianRamseteCommand command1 = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

        
        Trajectory trajectory2 = PathPlanner.loadPath("ThreeBallAuto-2", Units.feetToMeters(10), Units.feetToMeters(6), true);
        VitruvianRamseteCommand command2 = TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);
        
        addCommands(
            new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
            new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.FOLLOWER_COAST),
            // Turn on intake
            command1, // Pick up ball 1
            // Turn off intake
            // Shoot
            // Turn on intake
            command2 //Pick up ball 2
            // Turn off intkae
            // Shoot again
        ); 
    }
}
