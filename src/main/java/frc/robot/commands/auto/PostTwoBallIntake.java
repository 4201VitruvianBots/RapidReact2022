package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.intake.RunIntake;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.vitruvianlib.utils.TrajectoryUtils;

public class PostTwoBallIntake extends SequentialCommandGroup {
  /** Order of operations: Run intake Drive forward End sequence (end both) Retract intake */

  /**
   * Drives backward while running intake to pick up another ball after alignment during auto.
   *
   * @param driveTrain
   * @param fieldSim
   * @param indexer
   * @param intake
   */
public PostTwoBallIntake(DriveTrain driveTrain, FieldSim fieldSim, Indexer indexer, Intake intake) {
    Trajectory trajectory =
        PathPlanner.loadPath("TwoBallAuto-3", Units.feetToMeters(3), Units.feetToMeters(3), true);

    VitruvianRamseteCommand command =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);

    addCommands(
        // new SetOdometry(driveTrain, trajectory.getInitialPose()),
        new IntakePiston(intake, true),
        new ParallelDeadlineGroup(
            command.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new RunIntake(intake, indexer)),
        new IntakePiston(intake, false));
  }
}
