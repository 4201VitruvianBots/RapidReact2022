/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
  private Rotation2d currentVisionSetpoint = new Rotation2d();
  private Rotation2d currentTurretSetpoint = new Rotation2d();
  private Rotation2d currentRobotHeading = new Rotation2d();
  private Rotation2d lastVisionSetpoint = new Rotation2d();
  private Rotation2d lastTurretSetpoint = new Rotation2d();
  private Rotation2d lastRobotHeading = new Rotation2d();
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
    if (!m_climber.getElevatorClimbState()) {
      // If the turret is using sensor feedback
      if (m_turret.getTurretLocked()) {
        m_turret.setAbsoluteSetpointDegrees(0);
      } else if (m_turret.getControlMode() == Constants.CONTROL_MODE.CLOSEDLOOP) {
        currentRobotHeading = m_driveTrain.getHeadingRotation2d();
        joystickMoved =
            (Math.pow(m_controller.getRawAxis(0), 2) + Math.pow(m_controller.getRawAxis(1), 2))
                >= Math.pow(deadZone, 2);

        // if the joystick sensors report movement greater than the deadzone it runs these methods
        if (joystickMoved) {

          // Convert joystick axis values to degrees setpoint
          setpoint =
              -Units.radiansToDegrees(
                  Math.atan2(m_controller.getRawAxis(0), -m_controller.getRawAxis(1)));

          m_turret.setAbsoluteSetpointDegrees(setpoint);
        }
        // If we are estimating the hub's location using the pose of the hub and the robot
        else if (m_turret.usePoseEstimation()) {
          Translation2d goalToRobot =
              m_vision
                  .getHubPose()
                  .getTranslation()
                  .minus(m_driveTrain.getRobotPoseMeters().getTranslation());
          Rotation2d targetAngleRotation =
              new Rotation2d(Math.atan2(goalToRobot.getY(), goalToRobot.getX()));
          targetAngleRotation = targetAngleRotation.minus(m_driveTrain.getHeadingRotation2d());

          setpoint = targetAngleRotation.getDegrees();

          m_turret.setAbsoluteSetpointDegrees(setpoint);
          // if vision has a target and the joystick has not moved, set visionsetpoint to true and
          // run
          // the if statements below
        } else if (m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.LIMELIGHT)
            && !m_turret.usePoseEstimation()) {
          usingVisionSetpoint = true;
          m_vision.setLimelightLEDState(true);
          m_turret.setAbsoluteSetpointDegrees(
              m_turret.getTurretAngleDegrees()
                  - m_vision.getTargetXAngle(Constants.Vision.CAMERA_POSITION.LIMELIGHT));
        }
        // else if vision doesn't have a target and the joysticks have not moved,
        // set usingVisionSetpoint to false and turn off the LEDs if possible
        else if (!m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.LIMELIGHT)) {
          usingVisionSetpoint = false;
          m_vision.setLimelightLEDState(false);
        }

        currentTurretSetpoint = Rotation2d.fromDegrees(m_turret.getTurretAngleDegrees());

        // If no manual input is used and we're not using pose estimation, set the setpoint to the
        // vision setpoint
        // if (!joystickMoved && !m_turret.usePoseEstimation()) {
        //   setpoint =
        //       currentVisionSetpoint
        //           .plus(currentTurretSetpoint)
        //           .minus(currentRobotHeading.minus(lastRobotHeading))
        //           .getDegrees();

        //   m_turret.setAbsoluteSetpointDegrees(setpoint);
        // }

        // lastVisionSetpoint = currentVisionSetpoint;
        // lastTurretSetpoint = m_turret.getTurretRotation2d();
        // lastRobotHeading = m_driveTrain.getHeadingRotation2d();
      } else {
        m_turret.setPercentOutput(-m_controller.getRawAxis(0) * 0.2);
      }
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
