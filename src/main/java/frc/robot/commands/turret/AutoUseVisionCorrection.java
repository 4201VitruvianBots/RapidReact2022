/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.CAMERA_POSITION;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/** An example command that uses an example subsystem. */
public class AutoUseVisionCorrection extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;

  private final Vision m_vision;

  boolean turning, usingVisionSetpoint;
  private double setpoint;
  private double startTime;

  /** Creates a new ExampleCommand. */
  public AutoUseVisionCorrection(Turret turret, Vision vision) {
    m_turret = turret;
    m_vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.getValidTarget(CAMERA_POSITION.LIMELIGHT)) {
      setpoint =
          m_turret.getTurretAngleDegrees() + m_vision.getTargetXAngle(CAMERA_POSITION.LIMELIGHT);

      m_turret.setAbsoluteSetpointDegrees(setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_vision.getTargetXAngle(Constants.Vision.CAMERA_POSITION.LIMELIGHT))
        <= Constants.Flywheel.hubToleranceDegrees);
  }
}
