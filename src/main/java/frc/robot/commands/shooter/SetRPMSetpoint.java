// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;

public class SetRPMSetpoint extends CommandBase {
  private final Shooter m_shooter;
    private final Vision m_vision;
    private final double rpm;
  /** Creates a new SetRpmSetpoint. */
  public SetRPMSetpoint(Shooter shooter, Vision vision, double rpm) {
    m_shooter = shooter;
    m_vision = vision;
    this.rpm = rpm;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  /* m_vision.ledsOn();
   m_vision.setLastValidTargetTime();
   */
     m_shooter.setRPM(rpm); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setRPM(- 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
