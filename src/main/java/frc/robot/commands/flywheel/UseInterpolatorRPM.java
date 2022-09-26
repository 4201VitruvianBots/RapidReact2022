/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision.CAMERA_POSITION;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;

/** An example command that uses an example subsystem. */
public class UseInterpolatorRPM extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Flywheel m_flywheel;

  private final Vision m_vision;

  /** Creates a new ExampleCommand. */
  public UseInterpolatorRPM(Flywheel flywheel, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flywheel = flywheel;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setLimelightLEDState(true);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setRPM(
        () ->
            ShotSelecter.interpolateRPM(
                m_vision.getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
