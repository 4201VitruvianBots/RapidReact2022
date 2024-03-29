package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.InterruptingCommand;
import frc.robot.commands.driveTrain.CargoTrajectoryRameseteCommand;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.flywheel.UseInterpolatorRPM;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.AutoRunIntakeInstant;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.simulation.SetSimTrajectory;
import frc.robot.commands.simulation.SimulationShoot;
import frc.robot.commands.turret.AutoUseVisionCorrection;
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
public class TwoBallAuto extends SequentialCommandGroup {
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
  public TwoBallAuto(
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
        PathPlanner.loadPath("TwoBallAuto-1", Units.feetToMeters(8), Units.feetToMeters(7), true);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath("TwoBallAuto-2", Units.feetToMeters(8), Units.feetToMeters(7), true);

    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Command cargoVisionCommand = new CargoTrajectoryRameseteCommand(driveTrain, vision);
    /**
     * Order of operations: drivetrain & intake & indexer & vision run until drivetrain stops
     * (except for vision) run indexer & flywheel until indexer stops end sequence Turn and move
     * forward to line up with blue ball on other side of the line (NOT running intake, indexer,
     * shooter or vision) End path
     */
    new UseInterpolatorRPM(flywheel, vision).schedule();
    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1),
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        new IntakePiston(intake, true),
        new SetTurretAbsoluteSetpointDegrees(turret, 0),
        new WaitCommand(0.5),
        // new SetAndHoldRpmSetpoint(flywheel, vision, 1650),
        new UseInterpolatorRPM(flywheel, vision),
        new AutoRunIntakeInstant(intake, indexer, true),
        new InterruptingCommand(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            cargoVisionCommand.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            () -> false),
        new AutoRunIntakeInstant(intake, indexer, false),
        // new ParallelCommandGroup(
        new AutoUseVisionCorrection(turret, vision).withTimeout(1),
        new IntakePiston(intake, false),
        // TODO how long does flywheel take to rev up? (should the flywheel run while
        // driving?)
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel, 0.8).withTimeout(2),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new SetAndHoldRpmSetpoint(flywheel, vision, 0));
    // command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)));
  }
}
