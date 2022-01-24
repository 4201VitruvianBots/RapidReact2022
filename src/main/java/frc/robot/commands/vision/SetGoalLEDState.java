// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;

/** An example command that uses an example subsystem. */
public class SetGoalLEDState extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Vision m_vision;

  private boolean m_state;

  /**
   * Sets the state of the LEDs for the Goals
   *
   * @param vision The subsystem used by this command.
   * @param state State of the LEDs. Set to 'true' for 'on' and 'false' for 'off'.
   */
  public SetGoalLEDState(Vision vision, boolean state) {
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setGoalCameraLedState(m_state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
