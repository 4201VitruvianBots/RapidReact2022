/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class SetTurretSetpointFieldAbsolute extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;

  private final DriveTrain m_driveTrain;
  private final Vision m_vision;
  private final Flywheel m_flywheel;
  private final Climber m_climber;
  private final XboxController m_controller;
  private final double deadZone = 0.5;
  double setpoint;
  boolean timeout = false;
  boolean turning, usingVisionSetpoint;
  private boolean direction, directionTripped, joystickMoved;

  /** Creates a new ExampleCommand. */
  public SetTurretSetpointFieldAbsolute(
      Turret turretSubsystem,
      DriveTrain driveTrainSubsystem,
      Vision visionSubsystem,
      Flywheel flywheel,
      Climber climber,
      XboxController controller) {
    m_turret = turretSubsystem;
    m_driveTrain = driveTrainSubsystem;
    m_vision = visionSubsystem;
    m_flywheel = flywheel;
    m_climber = climber;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: Ask if hastarget=getGoalValidTarget

    // If we are not climbing
    if (!m_climber.getClimbState()) {
      //// If the turret is using sensor feedback
      if (m_turret.getControlMode() == Constants.CONTROL_MODE.CLOSEDLOOP) {
        // if the joystick sensors report movement greater than the deadzone it runs these methods
        if ((Math.pow(m_controller.getRawAxis(0), 2) + Math.pow(m_controller.getRawAxis(1), 2))
            >= Math.pow(deadZone, 2)) {
          // m_vision.setGoalCameraLedState(true);
          joystickMoved = true;

          // Convert joystick axis values to degrees setpoint
          setpoint =
              Math.toDegrees(Math.atan2(m_controller.getRawAxis(0), -m_controller.getRawAxis(1)));

          //// if vision has a target and the absolute value of the target is less than 20, make the
          // controller rumble
          if (m_vision.getGoalValidTarget()) {
            m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
            m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
          }
        }
        // if vision has a target and the joystick has not moved, set visionsetpoint to true and run
        // the if statements below
        else if (m_vision.getGoalValidTarget() && !joystickMoved) {
          usingVisionSetpoint = true;
          // if we are not turning, set leds to on and make the setpoint the turret angle combined
          // with vision targetx
          if (!turning) {
            m_vision.setGoalCameraLedState(true);
            setpoint = m_turret.getTurretAngle() + m_vision.getGoalTargetXAngle();
          }
          // if we are turning, set the led off and if the turret is on target, set turning
          // (variable) to false
          else {
            m_vision.setGoalCameraLedState(false);
            if (m_turret.onTarget()) turning = false;
          }
        }
        // else if vision doesn't have a target and the joysticks have not moved,
        // set usingvisionsetpoint to false and make the turret angle the setpoint
        else if (!m_vision.getGoalValidTarget() && !joystickMoved) {
          usingVisionSetpoint = false;
          setpoint = m_turret.getTurretAngle();
        }
        // else if we are not moving the joysticks at all, set directionTripped to false and set the
        // rumble of the controllers to zero
        else {
          directionTripped = false;
          joystickMoved = false;
          m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
          m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
        // if the shooter can shoot, set the rumble to 0.4 on both sides of the controller, else set
        // it to zero on both sides
        if (m_flywheel.canShoot()) {
          m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
          m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
        } else {
          m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
          m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
        // if the controlmode is 1, set the current setpoint as the turret setpoint
        // else set the percent output of the turret to the movement of the left joystick * 0.2
        m_turret.setAbsoluteSetpointDegrees(setpoint);
      } else {
        m_turret.setPercentOutput(m_controller.getRawAxis(0) * 0.2);
      }
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
}
