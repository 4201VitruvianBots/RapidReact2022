// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/** Sets the LED based on the subsystems' statuses */

// TODO: rewrite LED subsystem with comments aswell as redesigning the assigned
// colour values for each state

public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // TODO: make sure the correct subsystems are used
  private final LED m_led;

  private final Vision m_vision;
  private final Intake m_intake;
  private final Flywheel m_flywheel;
  private final Climber m_climber;
  // private final Controls m_controls;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
      LED led, Intake intake, Vision vision, Flywheel flywheel, Climber climber) {
    m_led = led;
    m_vision = vision;
    m_climber = climber;
    m_flywheel = flywheel;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.expressState(LED.robotState.Enabled);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // the prioritized state to be expressed to the LEDs
    // TODO: There should be disabled (not ready) and disabled (ready) like what we have on jango.
    boolean Disabled = DriverStation.isDisabled();
    boolean Enabled = true; // Reevaluate what is "Enabled"
    boolean Intaking = m_intake.getIntakeState();
    boolean VisionLock = m_vision.getGoalValidTarget();
    boolean Climbing = m_climber.getClimbState();

    // set in order of priority to be expressed from the least priority to the highest priority
    /*
     Disabled,
     Enabled,
     Intaking,
     VisionLock,
     Shooting,
     Climbing
    */
    if (Disabled) {
      m_led.expressState(LED.robotState.Disabled);
    }
    if (Enabled) {
      m_led.expressState(LED.robotState.Enabled);
    }
    if (Intaking) {
      m_led.expressState(LED.robotState.Intaking);
    }
    if (VisionLock) {
      m_led.expressState(LED.robotState.VisionLock);
    }
    if (Shooting) {
      m_led.expressState(LED.robotState.Shooting);
    }
    if (Climbing) {
      m_led.expressState(LED.robotState.Climbing);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
