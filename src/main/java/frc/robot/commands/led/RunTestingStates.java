// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/** Sets the LED based on the subsystems' statuses */
public class RunTestingStates extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final LED m_led;
  double lastRunTime = Timer.getFPGATimestamp();

  // private final Controls m_controls;

  /** Sets the LED based on the subsystems' statuses */
  public RunTestingStates(LED led) {
    m_led = led;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean runTheTask;

    double curTime = Timer.getFPGATimestamp();
    double timeSinceLastRun = curTime - lastRunTime;

    if (timeSinceLastRun >= 3.0) {
      runTheTask = true;
    } else {
      runTheTask = false;
    }

    if (runTheTask) {
      m_led.increaseTestingState(true);
      lastRunTime = curTime;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

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