package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.AutoRunIntakeIndexer;
import frc.robot.commands.intake.AutoRunIntakeInstant;
import frc.robot.commands.intake.AutoRunIntakeOnly;
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
public class FourBallAuto extends SequentialCommandGroup {
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
  public FourBallAuto(
      DriveTrain driveTrain,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {

    Trajectory trajectory1 =
        PathPlanner.loadPath("FourBallAuto-1", Units.feetToMeters(9), Units.feetToMeters(4), true);
    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath("FourBallAuto-2", Units.feetToMeters(9), Units.feetToMeters(4), true);
    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Trajectory trajectory3 =
        PathPlanner.loadPath(
            "FourBallAuto-3", Units.feetToMeters(12), Units.feetToMeters(10), false);
    VitruvianRamseteCommand command3 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory3);

    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1),
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),

        // INTAKE 1
        new IntakePiston(intake, true),
        new SetTurretAbsoluteSetpointDegrees(turret, 30),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1625),
        new ParallelDeadlineGroup(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoRunIntakeIndexer(intake, indexer)),
        new IntakePiston(intake, false),

        // SHOOT 1
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel, 0.8).withTimeout(0.9),
            new SimulationShoot(fieldSim, true).withTimeout(0.9),
            RobotBase::isReal),

        // INTAKE 2
        new SetAndHoldRpmSetpoint(flywheel, vision, 1700),
        new SetTurretAbsoluteSetpointDegrees(turret, 30),
        new IntakePiston(intake, true),
        new AutoRunIntakeInstant(intake, indexer, true),
        command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        new AutoRunIntakeIndexer(intake, indexer).withTimeout(1),
        new IntakePiston(intake, false),

        // SHOOT 2
        command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
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
