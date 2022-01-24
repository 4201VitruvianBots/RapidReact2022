/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretControlMode;
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
    //// if the turret is using sensor feedback
    if (m_turret.getControlMode() == TurretControlMode.CLOSEDLOOP) {
      // if vision has a valid target to shoot at then set usingVisinoSetpoint to true
      if (m_vision.getGoalValidTarget()) {
        usingVisionSetpoint = true;
        // if we are not turning then turn on vision leds and set the turret setpoint to the turret
        // angle + targetx
        if (!turning) {
          m_vision.setGoalCameraLedState(true);
          setpoint = m_turret.getTurretAngle() + m_vision.getGoalTargetXAngle();
          // if the setpoint is greater than the max turret angle then subtract 360 from it
          if (setpoint > m_turret.getMaxAngle()) {
            setpoint -= 360;
            // if the setpiont is less than the minimum angle of the turret, make the setpoint the
            // minimum angle and set turning to true
            if (setpoint < m_turret.getMinAngle()) setpoint = m_turret.getMinAngle();
            turning = true;
            // else if the setpoint is less than the minimum angle, add 360 to it
          } else if (setpoint < m_turret.getMinAngle()) {
            setpoint += 360;
            // if the setpoint is greater than the turret's max angle, make it the max angle and set
            // turning to true
            if (setpoint > m_turret.getMaxAngle()) setpoint = m_turret.getMaxAngle();
            turning = true;
          }
        }
        // else if the turret is on target, set turning to false
        else {
          if (m_turret.onTarget()) turning = false;
        }
        // else if vision does not have a valid target, set usingVisionSetpoint to false and make
        // the setpoint the current turret angle
      } else if (!m_vision.getGoalValidTarget()) {
        usingVisionSetpoint = false;
        setpoint = m_turret.getTurretAngle();
      }

      m_turret.setRobotCentricSetpoint(setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_vision.getGoalTargetXAngle()) <= 3);
  }
}
