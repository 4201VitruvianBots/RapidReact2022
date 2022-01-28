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
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
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

/** Shoots one cargo, intakes and shoots a second, then intakes and shoots a third. */
public class IndividualThreeBallAuto extends SequentialCommandGroup {
  /**
   * Shoots one cargo, intakes and shoots a second, then intakes and shoots a third.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   * @param intake Runs the intake to pick up new cargo.
   * @param indexer Feeds cargo through indexer to shoot.
   * @param flywheel Rev flywheel to shoot.
   * @param turret Turn turret to goal.
   * @param vision Find target.
   */
  public IndividualThreeBallAuto(
    DriveTrain driveTrain,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {
    /**
     * Shoots cargo the ball started with
     * Drives backwards, intake running, picks up second cargo
     * Shoots second cargo
     * Drives backward, intake running, picks up third cargo
     * Shoots third cargo
     * Drives back to the terminal and lines up with the ball there, to pick up at the start of tele-op
     */


    Trajectory trajectory1 =
        PathPlanner.loadPath("ThreeBallAuto-1", Units.feetToMeters(2), Units.feetToMeters(2), true);
    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath("ThreeBallAuto-2", Units.feetToMeters(4), Units.feetToMeters(4), true);
    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Trajectory trajectory3 =
        PathPlanner.loadPath("ThreeBallAuto-3", Units.feetToMeters(4), Units.feetToMeters(4), true);
    VitruvianRamseteCommand command3 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory3);

    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.HALF_BRAKE),
        new SetIntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 3000),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new FeedAll(indexer),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new ParallelDeadlineGroup(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoControlledIntake(intake, indexer)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new FeedAll(indexer),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new ParallelDeadlineGroup(
            command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoControlledIntake(intake, indexer)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new FeedAll(indexer),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)));  
  }
}
