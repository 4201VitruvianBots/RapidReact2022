package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.InterruptingCommand;
import frc.robot.commands.driveTrain.CargoTrajectoryRameseteCommand;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.*;
import frc.robot.commands.simulation.SetSimTrajectory;
import frc.robot.commands.simulation.SimulationShoot;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Intakes one cargo and shoots two cargo into the high goal. */
public class FiveBallAutoLongShotVision extends SequentialCommandGroup {
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
  public FiveBallAutoLongShotVision(
      DriveTrain driveTrain,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {

    Trajectory trajectory1 =
        PathPlanner.loadPath("FiveBallAuto-1", Units.feetToMeters(8), Units.feetToMeters(7), true);
    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath("FiveBallAuto-2", Units.feetToMeters(7), Units.feetToMeters(6), false);
    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Trajectory trajectory3 =
        PathPlanner.loadPath(
            "FiveBallAuto-3", Units.feetToMeters(11), Units.feetToMeters(10), true);
    VitruvianRamseteCommand command3 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory3);

    Trajectory trajectory4 =
        PathPlanner.loadPath(
            "FiveBallAuto-4", Units.feetToMeters(11), Units.feetToMeters(10), false);
    VitruvianRamseteCommand command4 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory4);

    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1),
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),

        // INTAKE 1
        new IntakePiston(intake, true),
        new SetTurretAbsoluteSetpointDegrees(turret, 0),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1625),
        new AutoRunIntakeInstant(intake, indexer, true),
        new InterruptingCommand(
                command1, new CargoTrajectoryRameseteCommand(driveTrain, vision), () -> false)
            .andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        new AutoRunIntakeInstant(intake, indexer, false),
        new IntakePiston(intake, false),

        // SHOOT 1
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel, 0.8).withTimeout(0.9),
            new SimulationShoot(fieldSim, true).withTimeout(0.9),
            RobotBase::isReal),

        // INTAKE 1
        new IntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1875),
        new SetTurretAbsoluteSetpointDegrees(turret, 15),
        new AutoRunIntakeInstant(intake, indexer, true),
        new InterruptingCommand(
                command2,
                new CargoTrajectoryRameseteCommand(driveTrain, vision),
                () -> vision.cargoInRangeWithPositionCheck(Constants.Vision.CARGO_TARMAC_TWO))
            .andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        new AutoRunIntakeInstant(intake, indexer, false),
        new IntakePiston(intake, false),

        // SHOOT 2
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel, 1.4).withTimeout(0.9),
            new SimulationShoot(fieldSim, true).withTimeout(0.9),
            RobotBase::isReal),
        // INTAKE 2
        new SetAndHoldRpmSetpoint(flywheel, vision, 1875),
        new SetTurretAbsoluteSetpointDegrees(turret, 10),
        new IntakePiston(intake, true),
        new AutoRunIntakeInstant(intake, indexer, true),
        new InterruptingCommand(
                command3,
                new CargoTrajectoryRameseteCommand(driveTrain, vision),
                () -> vision.cargoInRangeWithPositionCheck(Constants.Vision.CARGO_TERMINAL))
            .andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        new AutoRunIntakeIndexer(intake, indexer).withTimeout(1),
        new IntakePiston(intake, false),

        // SHOOT 3
        new ParallelDeadlineGroup(
            command4.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoRunIndexer(indexer, flywheel, -0.8, true).withTimeout(0.2)),
        new IntakePiston(intake, false),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.75),
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new AutoRunIndexer(indexer, flywheel, 0.80).withTimeout(5.0),
                new SimulationShoot(fieldSim, true).withTimeout(5.0),
                RobotBase::isReal),
            new AutoRunIntakeOnly(intake)));
  }
}

/**
 * drive forward while intaking shoot [SHOOT 1] drive backwards [curve] whilie intaking, back to
 * terminal drive forwards while intaking shoot while intaking [SHOOT 2] shoot until last ball is
 * shot
 */
