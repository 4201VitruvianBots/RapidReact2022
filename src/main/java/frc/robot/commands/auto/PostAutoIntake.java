package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.DriveBackwardDistance;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.intake.RunIntake;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class PostAutoIntake extends SequentialCommandGroup {
  /** Order of operations: Run intake Drive forward End sequence (end both) Retract intake */

  /**
   * Drives backward while running intake to pick up another ball after alignment during auto.
   *
   * @param driveTrain
   * @param fieldSim
   * @param indexer
   * @param intake
   */
  public PostAutoIntake(DriveTrain driveTrain, FieldSim fieldSim, Indexer indexer, Intake intake) {
    addCommands(
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.HALF_BRAKE),
        new IntakePiston(intake, true),
        new ParallelDeadlineGroup(
            new DriveBackwardDistance(driveTrain, fieldSim, 2)
                .andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new RunIntake(intake, indexer)),
        new IntakePiston(intake, false));
  }
}
