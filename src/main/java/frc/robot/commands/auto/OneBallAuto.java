package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.simulation.SetSimTrajectory;
import frc.robot.commands.simulation.SimulationShoot;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Drives off the tarmac and shoots two cargo into the high goal. */
public class OneBallAuto extends SequentialCommandGroup {
  /**
   * Drives off the tarmac and shoots two cargo into the high goal.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   */
  public OneBallAuto(
      DriveTrain driveTrain,
      FieldSim fieldSim,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {
    // Drive backward
    // Stop and aim for high goal
    // Shoot 1 cargo into high goal
    Trajectory trajectory1 =
        PathPlanner.loadPath("OneBallAuto-1", Units.feetToMeters(2), Units.feetToMeters(2), true);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath("OneBallAuto-2", Units.feetToMeters(2), Units.feetToMeters(2), true);

    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1, trajectory2),
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        new SetAndHoldRpmSetpoint(flywheel, vision, 3000),
        command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel).withTimeout(0.5),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal));
    command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0));
    // Rev up flywheel while driving backwards
    // Once finish driving, feed indexer
    // After that, stop indexer and flywheel
  }
}
