// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/** Sets the LED based on the subsystems' statuses */
public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LED m_led;

  private final Flywheel m_flywheel;
  private final Intake m_intake;
  private final Climber m_climber;
  private final Indexer m_indexer;
  private boolean disabled;
  private boolean enabled;
  private boolean intaking;
  private boolean canShoot;
  private boolean climbing;
  private boolean opponentBall;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
      LED led, Intake intake, Flywheel flywheel, Climber climber, Indexer indexer) {
    m_led = led;
    m_climber = climber;
    m_intake = intake;
    m_flywheel = flywheel;
    m_indexer = indexer;
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
    disabled = DriverStation.isDisabled();
    enabled = !disabled;
    intaking = m_intake.getIntakeState();
    canShoot = m_flywheel.canShoot();
    climbing = m_climber.getElevatorClimbState();
    opponentBall = m_indexer.hasOpponentBall();

    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (disabled) {
      m_led.expressState(LED.robotState.Disabled);
    } else if (climbing) {
      m_led.expressState(LED.robotState.Climbing);
    } else if (canShoot) {
      m_led.expressState(LED.robotState.CanShoot);
    } else if (intaking) {
      m_led.expressState(LED.robotState.Intaking);
    } else if (opponentBall) {
      m_led.expressState(LED.robotState.OpponentBall);
    } else if (enabled) {
      m_led.expressState(LED.robotState.Enabled);
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
