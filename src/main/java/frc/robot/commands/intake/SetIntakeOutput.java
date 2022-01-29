// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class SetIntakeOutput extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;

  private final double m_output;

  /**
   * Creates a new SetIntake
   *
   * @param intake The intake used by this command
   * @param output creates an instance of the subsystem
   */
  public SetIntakeOutput(Intake intake, double output) {
    m_intake = intake;
    m_output = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  /** Called when the command is initially scheduled. Sets the Intake state to true */
  @Override
  public void initialize() {
    m_intake.setIntakeState(true);
  }

  /** Called every time the scheduler runs while the command is scheduled. Sets the Intake speed */
  @Override
  public void execute() {
    m_intake.setIntakePercentOutput(m_output);
  }

  /**
   * Called once the command ends or is interrupted. Sets the speed of the Intake to 0 and sets the
   * intake state to false
   */
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePercentOutput(0);
    m_intake.setIntakeState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}