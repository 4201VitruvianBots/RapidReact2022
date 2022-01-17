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
  // private final Controls m_controls;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(LED led, Vision vision) {
    m_led = led;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.expressState(LED.robotState.READY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // the prioritized state to be expressed to the LEDs
    LED.robotState expressedRobotState = null;

    // TODO: correct and expand to other subsystems and commands
    // the possible states that can be passed to the LEDs and their values
    boolean driverStationReady = !DriverStation.isDisabled();
    boolean visionHasTarget = /*m_vision.hasTarget();*/ true;
    boolean shooterIsActive = /*turret.shooterIsActive();*/ true;

    // set in order of priority to be expressed from the least priority to the highest priority
    if (!driverStationReady) {
      expressedRobotState = LED.robotState.NOPE;
    }
    if (driverStationReady) {
      expressedRobotState = LED.robotState.READY;
    }
    if (visionHasTarget) {
      expressedRobotState = LED.robotState.SET;
    }
    if (shooterIsActive) {
      expressedRobotState = LED.robotState.GO;
    }

    m_led.expressState(expressedRobotState);
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
