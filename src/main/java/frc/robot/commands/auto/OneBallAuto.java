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
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision; 
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Drives off the tarmac and shoots one cargo into the high goal. */
public class OneBallAuto extends SequentialCommandGroup {
  /**
   * Drives off the tarmac and shoots one cargo into the high goal.
   *
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   * @param indexer To feed the cargo to the shooter.
   * @param flywheel To shoot the cargo.
   * @param vision To find the target.
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
    Trajectory trajectory =
      PathPlanner.loadPath("OneBallAuto", Units.feetToMeters(2), Units.feetToMeters(2), true);

    VitruvianRamseteCommand command =
      TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory);

    addCommands(
      new SetOdometry(driveTrain, fieldSim, trajectory.getInitialPose()),
      new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.HALF_BRAKE),
      new SetAndHoldRpmSetpoint(flywheel, vision, 3000),
      command.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
      new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
      new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
      new ConditionalCommand(
        new FeedAll(indexer),
        new SimulationShoot(fieldSim, true).withTimeout(2),
        RobotBase::isReal
      )
        // Rev up flywheel while driving backwards
        // Once finish driving, feed indexer
        // After that, stop indexer and flywheel
    );
  }
}
