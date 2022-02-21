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
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.turret.AutoUseVisionCorrection;
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
 * Intakes one cargo and shoots two cargo into the high goal.
 * Starting from the lower part of the field 
 */
public class TwoBallAutoLower extends SequentialCommandGroup {
  /**
   * Intakes one cargo and shoots two cargo into the high goal.
   * Starting from the lower part of the field 
   * 
   * 
   * @param driveTrain The driveTrain used by this command.
   * @param fieldSim The fieldSim used by this command.
   * @param intake Runs the intake to pick up new cargo.
   * @param indexer Feeds cargo through indexer to shoot.
   * @param flywheel Rev flywheel to shoot.
   * @param turret Turn turret to goal.
   * @param vision Find target.
   */
  public TwoBallAutoLower(
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
        PathPlanner.loadPath("TwoBallAuto-Lower", Units.feetToMeters(8), Units.feetToMeters(7), true);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);



    /**
     * Order of operations: drivetrain & intake & indexer & vision run until drivetrain stops
     * (except for vision) run indexer & flywheel until indexer stops end sequence Turn and move
     * forward to line up with blue ball on other side of the line (NOT running intake, indexer,
     * shooter or vision) End path
     */
    addCommands(
        new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        new IntakePiston(intake, true),
        new SetTurretAbsoluteSetpointDegrees(turret, 5),
        new WaitCommand(0.5),
        new SetAndHoldRpmSetpoint(flywheel, vision, 2400),
        new ParallelDeadlineGroup(
            command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            new AutoRunIntake(intake)
            // TODO implement indexer
            ),
        new AutoRunIntake(intake).withTimeout(1),
        new IntakePiston(intake, false),
        new AutoUseVisionCorrection(turret, vision).withTimeout(1),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        // TODO how long does flywheel take to rev up? (should the flywheel run while
        // driving?)
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel).withTimeout(4),
            new SimulationShoot(fieldSim, true).withTimeout(2),
            RobotBase::isReal),
        new SetAndHoldRpmSetpoint(flywheel, vision, 0));
       // command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
       // new SetAndHoldRpmSetpoint(flywheel, vision, 0));
    //    new SchedulePostAutoCommand(.
    //         driveTrain, new PostTwoBallIntake(driveTrain, fieldSim, indexer, intake)));
  }
}
