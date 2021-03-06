package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.AutoRunIntake;
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

/** Intakes one cargo, shoots two, then intakes and shoots a third cargo */
public class ThreeBallAuto extends SequentialCommandGroup {
  /**
   * Intakes one cargo, shoots two, then intakes and shoots a third cargo
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   * @param intake Runs the intake to pick up new cargo.
   * @param indexer Feeds cargo through indexer to shoot.
   * @param flywheel Rev flywheel to shoot.
   * @param turret Turn turret to goal.
   * @param vision Find target.
   */
  public ThreeBallAuto(
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
        PathPlanner.loadPath("ThreeBallAuto-1", Units.feetToMeters(7), Units.feetToMeters(7), true);
    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath(
            "ThreeBallAuto-2", Units.feetToMeters(7), Units.feetToMeters(6), false);
    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Trajectory trajectory3 =
        PathPlanner.loadPath("ThreeBallAuto-3", Units.feetToMeters(7), Units.feetToMeters(6), true);
    VitruvianRamseteCommand command3 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory3);

    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1, trajectory2, trajectory3),
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        new IntakePiston(intake, true),
        new ParallelCommandGroup(
            new SetTurretAbsoluteSetpointDegrees(turret, -5), new WaitCommand(.5)),
        new SetAndHoldRpmSetpoint(flywheel, vision, 2400),
        new ParallelDeadlineGroup(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoRunIntake(intake, indexer)),
        new AutoRunIntake(intake, indexer).withTimeout(1),
        new AutoUseVisionCorrection(turret, vision).withTimeout(1),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel).withTimeout(1),
            new SimulationShoot(fieldSim, true).withTimeout(1),
            RobotBase::isReal),
        new SetAndHoldRpmSetpoint(flywheel, vision, 2400),
        command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        new ParallelDeadlineGroup(
            command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoRunIntake(intake, indexer),
            new SetTurretAbsoluteSetpointDegrees(turret, 60)),
        new AutoRunIntake(intake, indexer).withTimeout(1),
        new IntakePiston(intake, false),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel).withTimeout(1),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new SetAndHoldRpmSetpoint(flywheel, vision, 0));
    // command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)));
  }
  // class ToastAuto {
}
