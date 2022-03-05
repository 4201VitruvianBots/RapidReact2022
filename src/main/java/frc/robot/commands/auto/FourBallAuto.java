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
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

/**
 * Intakes one cargo, shoots two, then intakes and shoots two more cargo, one of them coming from
 * the terminal
 */
public class FourBallAuto extends SequentialCommandGroup {
  /**
   * Intakes one cargo, shoots two, then intakes and shoots two more cargo, one of them coming from
   * the terminal
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
    /**
     * Shoots cargo the ball started with Drives backwards, intake running, picks up second cargo
     * Shoots second cargo Drives backward, intake running, picks up third cargo Shoots third cargo
     * Drives back to the terminal and lines up with the ball there, to pick up at the start of
     * tele-op
     */
    Trajectory trajectory1 =
        PathPlanner.loadPath("FourBallAuto-1", Units.feetToMeters(12), Units.feetToMeters(6), true);
    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath(
            "FourBallAuto-2", Units.feetToMeters(12), Units.feetToMeters(6), false);
    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Trajectory trajectory3 =
        PathPlanner.loadPath("FourBallAuto-3", Units.feetToMeters(14), Units.feetToMeters(8), true);
    VitruvianRamseteCommand command3 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory3);

    Trajectory trajectory4 =
        PathPlanner.loadPath(
            "FourBallAuto-4", Units.feetToMeters(14), Units.feetToMeters(8), false);
    VitruvianRamseteCommand command4 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory4);

    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        new IntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1800),
        new SetTurretAbsoluteSetpointDegrees(turret, 0),
        new ParallelDeadlineGroup(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new RunIntake(intake, indexer)),
        new RunIntake(intake, indexer).withTimeout(1),
        new IntakePiston(intake, false),
        command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel).withTimeout(1),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1800),
        new ParallelDeadlineGroup(
            command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new RunIntake(intake, indexer)),
        new SetTurretAbsoluteSetpointDegrees(turret, 0),
        command4.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel).withTimeout(2),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new SetAndHoldRpmSetpoint(flywheel, vision, 0));
    // command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)));
  }
  // class ToastAuto {
}
