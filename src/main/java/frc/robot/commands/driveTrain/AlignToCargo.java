/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.driveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class AlignToCargo extends CommandBase {

  private final double P_TERM = 0.016;
  private final double I_TERM = 0;
  private final double D_TERM = 0.0;

  private final DriveTrain m_driveTrain;
  private final Vision m_vision;
  private final DoubleSupplier m_throttle;
  private final DoubleSupplier m_turn;
  private final PIDController pid = new PIDController(P_TERM, I_TERM, D_TERM);

  public AlignToCargo(
      DriveTrain driveTrain, Vision vision, DoubleSupplier throttle, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveTrain = driveTrain;
    this.m_vision = vision;
    this.m_throttle = throttle;
    this.m_turn = turn;
    addRequirements(this.m_driveTrain);
    addRequirements(this.m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickY = (Math.abs(m_throttle.getAsDouble()) > 0.05) ? m_throttle.getAsDouble() : 0;
    double joystickX = (Math.abs(m_turn.getAsDouble()) > 0.05) ? m_turn.getAsDouble() : 0;

    double throttle = joystickY;
    double turn = joystickX;
    if (m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.INTAKE)) {
      double setpoint =
          m_driveTrain.getHeadingDegrees()
              + m_vision.getTargetXAngle(Constants.Vision.CAMERA_POSITION.INTAKE);

      double turnAdjustment = pid.calculate(m_driveTrain.getHeadingDegrees(), setpoint);

      turn = turnAdjustment;
      //            turn += Math.max(Math.min(turnAdjustment, 0.6), -0.6);
    }

    m_driveTrain.setMotorArcadeDrive(throttle, turn);
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
