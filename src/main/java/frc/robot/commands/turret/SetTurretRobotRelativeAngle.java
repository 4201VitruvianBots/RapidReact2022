// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/** An example command that uses an example subsystem. */
public class SetTurretRobotRelativeAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;

  private final double m_setpoint;
  private double startTime;

  public SetTurretRobotRelativeAngle(Turret turret, double setpoint) {
    m_turret = turret;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_turret.setRobotCentricSetpoint(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.

  /** @return absolute value of turret angle minus that b m_setpoint and add a <1 at the end */
  @Override
  public boolean isFinished() {
    return false;
  }
}
