package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.InterruptingCommand;
import frc.robot.commands.driveTrain.DriveToCargoTrajectory;
import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
import frc.robot.commands.driveTrain.SetOdometry;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.intake.AutoRunIntakeIndexer;
import frc.robot.commands.intake.AutoRunIntakeOnly;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.simulation.SetSimTrajectory;
import frc.robot.commands.simulation.SimulationShoot;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

/** Intakes one cargo and shoots two cargo into the high goal. */
public class FiveBallAutoNew extends SequentialCommandGroup {
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
  public FiveBallAutoNew(
      DriveTrain driveTrain,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {

    Trajectory trajectory1 =
        PathPlanner.loadPath(
            "FiveBallAuto-1", Units.feetToMeters(8), Units.feetToMeters(7), true);
    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    Trajectory trajectory2 =
        PathPlanner.loadPath(
            "FiveBallAuto-2", Units.feetToMeters(8), Units.feetToMeters(7), false);
    VitruvianRamseteCommand command2 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory2);

    Trajectory trajectory3 =
        PathPlanner.loadPath(
            "FiveBallAuto-3", Units.feetToMeters(8), Units.feetToMeters(7), true);
    VitruvianRamseteCommand command3 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory3);

    Trajectory trajectory4 =
        PathPlanner.loadPath(
            "FiveBallAuto-4", Units.feetToMeters(8), Units.feetToMeters(7), false);
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
        new ParallelDeadlineGroup(
            new InterruptingCommand(
                command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)), 
                new DriveToCargoTrajectory(driveTrain,vision),
                vision::cargoInRange),
            new AutoRunIntakeIndexer(intake, indexer)),
        new IntakePiston(intake, false),

        // SHOOT 2
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new ConditionalCommand(
            new AutoRunIndexer(indexer, flywheel, 0.8).withTimeout(0.9),
            new SimulationShoot(fieldSim, true).withTimeout(0.9),
            RobotBase::isReal),

        // INTAKE 2
        new IntakePiston(intake, true),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new InterruptingCommand(
                    command2.andThen(() -> driveTrain.setMotorTankDrive(0, 0)), 
                    new DriveToCargoTrajectory(driveTrain,vision),
                    vision::cargoInRange),
               new SetAndHoldRpmSetpoint(flywheel, vision, 1875),
               new SetTurretAbsoluteSetpointDegrees(turret, -6),
               new InterruptingCommand(
                    command3.andThen(() -> driveTrain.setMotorTankDrive(0, 0)), 
                    new DriveToCargoTrajectory(driveTrain,vision),
                    vision::cargoInRange)),
            new AutoRunIntakeIndexer(intake, indexer)),
            new IntakePiston(intake, false),

        // SHOOT 3 
        new ParallelDeadlineGroup(
             command4.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
             new AutoRunIndexer (indexer, flywheel,-0.8,true).withTimeout(0.2)),
        new IntakePiston(intake, true),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
        new ParallelDeadlineGroup( 
            new ConditionalCommand(
                new AutoRunIndexer(indexer, flywheel, 0.80).withTimeout(5.0),
                new SimulationShoot(fieldSim, true).withTimeout(5.0),
                RobotBase::isReal),
            new AutoRunIntakeOnly(intake))
    );
  }
}


/**
 * drive forward while intaking 
 * shoot [SHOOT 1]
 * drive backwards [curve] whilie intaking, back to terminal
 * drive forwards while intaking
 * shoot while intaking [SHOOT 2]
 * shoot until last ball is shot
 **/ 
