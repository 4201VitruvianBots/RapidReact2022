package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.RobotContainer;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Intakes one cargo and shoots two cargo into the high goal. */
public class TwoBallAutoDefense extends SequentialCommandGroup {
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
   * @param robotContainer to run post auto intake.
   */
  public TwoBallAutoDefense(
      DriveTrain driveTrain,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision,
      RobotContainer robotContainer) {
    // Drive backward maximum distance to ball
    // While dirivng backward, intake is running
    // Stop (now with 2 cargo) and aim for high goal
    // Shoot 2 cargo into high goal

    Trajectory trajectory1 =
        PathPlanner.loadPath("TwoBallAuto", Units.feetToMeters(4), Units.feetToMeters(4), true);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath("TwoBallAuto-2", Units.feetToMeters(4), Units.feetToMeters(4), true);

    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    /**
     * Order of operations: drivetrain & intake & indexer & vision run until drivetrain stops
     * (except for vision) run indexer & flywheel until indexer stops end sequence Turn and move
     * forward to line up with blue ball on other side of the line (NOT running intake, indexer,
     * shooter or vision) End path
     */
    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.HALF_BRAKE),
        new IntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 3000),
        new ParallelDeadlineGroup(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new RunIntake(intake, indexer)
            // TODO implement indexer
            ),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        // TODO how long does flywheel take to rev up? (should the flywheel run while
        // driving?)
        new ConditionalCommand(
            new RunIndexer(indexer).withTimeout(1),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new ParallelDeadlineGroup(
                command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
                new RunIntake(intake, indexer))
            .withTimeout(5),
        new ConditionalCommand(
            new RunIndexer(indexer).withTimeout(1),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new InstantCommand().andThen(robotContainer::setAutoAligned));
  }
}
