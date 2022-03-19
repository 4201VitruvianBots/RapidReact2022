package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.DriveToCargoTrajectory;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.AutoRunIntakeIndexer;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.simulation.SetSimTrajectory;
import frc.robot.commands.simulation.SimulationShoot;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Intakes one cargo and shoots two cargo into the high goal. */
public class FiveBallAutoRed extends SequentialCommandGroup {
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
  public FiveBallAutoRed(
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

    Trajectory trajectory1 =
        PathPlanner.loadPath(
            "FiveBallAutoRed-1", Units.feetToMeters(8), Units.feetToMeters(7), true);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath(
            "FiveBallAutoRed-2", Units.feetToMeters(8), Units.feetToMeters(7), false);

    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Trajectory trajectory3 =
        PathPlanner.loadPath(
            "FiveBallAutoRed-3", Units.feetToMeters(8), Units.feetToMeters(7), true);

    VitruvianRamseteCommand command3 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory3);

    Trajectory trajectory4 =
        PathPlanner.loadPath(
            "FiveBallAutoRed-4", Units.feetToMeters(14), Units.feetToMeters(5), false);

    VitruvianRamseteCommand command4 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory4);

    Trajectory trajectory5 =
        PathPlanner.loadPath(
            "FiveBallAutoRed-5", Units.feetToMeters(14), Units.feetToMeters(5), false);

    VitruvianRamseteCommand command5 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory5);

    /**
     * Order of operations: drivetrain & intake & indexer & vision run until drivetrain stops
     * (except for vision) run indexer & flywheel until indexer stops end sequence Turn and move
     * forward to line up with blue ball on other side of the line (NOT running intake, indexer,
     * shooter or vision) End path
     */
    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1, trajectory2, trajectory3, trajectory4),
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        new IntakePiston(intake, true),
        new SetTurretAbsoluteSetpointDegrees(turret, 0),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1650),
        new WaitCommand(0.5),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
                new DriveToCargoTrajectory(driveTrain, vision),
                command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
                new DriveToCargoTrajectory(driveTrain, vision)),
            new SequentialCommandGroup(
                // new WaitCommand(0.25),
                new ConditionalCommand(
                    new AutoRunIndexer(indexer, flywheel, 0.90)
                        .withTimeout(1), // if this still overshoots, raise some more (0.95ish)
                    new SimulationShoot(fieldSim, true).withTimeout(1),
                    RobotBase::isReal)),
            new AutoRunIntakeIndexer(intake, indexer)),
        new IntakePiston(intake, false),
        new SetTurretAbsoluteSetpointDegrees(turret, 40),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1800),
        command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel).withTimeout(0.9),
            new SimulationShoot(fieldSim, true).withTimeout(0.9),
            RobotBase::isReal),
        new IntakePiston(intake, true),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                command4.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
                new DriveToCargoTrajectory(driveTrain, vision)),
            new AutoRunIntakeIndexer(intake, indexer)),
        new IntakePiston(intake, false),
        new SetTurretAbsoluteSetpointDegrees(turret, 15),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1850),
        new ParallelDeadlineGroup(
            command5.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoRunIntakeIndexer(intake, indexer).withTimeout(1)),
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel, 0.85),
            new SimulationShoot(fieldSim, true),
            RobotBase::isReal));
  }
}
