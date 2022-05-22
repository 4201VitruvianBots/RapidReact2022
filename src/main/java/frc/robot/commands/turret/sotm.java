/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class sotm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;

  private final DriveTrain m_driveTrain;
  private final Vision m_vision;
  private final Flywheel m_flywheel;
  private final double HorizontalDistanceOffset = 0.0;

  private double hubDistance = m_vision.getGoalTargetHorizontalDistance(Constants.Vision.CAMERA_POSITION.LIMELIGHT) + HorizontalDistanceOffset;
  private double hubAngle = m_vision.getTargetXAngle(Constants.Vision.CAMERA_POSITION.LIMELIGHT);
  private double robotVelocity = 0; // instantanious velocity of the robot in meters per second
  private double ballTimeInAir = 0; // the time that the ball will spend in the air
  // TODO: solve the paradox of finding out how long the ball will be in the air

  /** Creates a new sotm. */
  public sotm(
      Turret turretSubsystem,
      DriveTrain driveTrainSubsystem,
      Vision visionSubsystem,
      Flywheel flywheel) {
    m_turret = turretSubsystem;
    m_driveTrain = driveTrainSubsystem;
    m_vision = visionSubsystem;
    m_flywheel = flywheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
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
