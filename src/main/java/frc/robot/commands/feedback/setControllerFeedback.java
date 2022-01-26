// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feedback;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision;

/** An example command that uses an example subsystem. */
public class setControllerFeedback extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Vision m_vision;
  private final Joystick m_controller;

  /**
   * Creates a new setControllerFeedback.
   *
   * @param vision The subsystem used by this command.
   */
  public setControllerFeedback(Vision vision, Joystick controller) {
    m_vision = vision;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_vision.canShoot()){
      m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
      m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
    } else {
      m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
      m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
