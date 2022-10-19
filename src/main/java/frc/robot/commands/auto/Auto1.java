// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.simulation.SetSimTrajectory;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.TrajectoryUtils;

public class Auto1 extends SequentialCommandGroup {

  /** Creates a new Auto1. */
  public Auto1(DriveTrain driveTrain, FieldSim fieldSim) {

    // Use addRequirements() here to declare subsystem dependencies.

    Trajectory trajectory1 =
        PathPlanner.loadPath("MyVeryCoolPath", Units.feetToMeters(7), Units.feetToMeters(7), true);

    VitruvianRamseteCommand command1 =
        TrajectoryUtils.generateRamseteCommand(driveTrain, trajectory1);

    addCommands(
        new SetSimTrajectory(fieldSim, trajectory1),
        command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
