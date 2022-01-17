// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetIntakePiston extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  boolean extend;

  /**
   * @param intake The intake used by this command.
   * @return returns true or false if the piston is extended
   */
  public SetIntakePiston(Intake intake, boolean extend) {
        m_intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        this.extend = extend;
    }

    /**
     *  Called when the command is initially scheduled.
     *  sets the intake piston to either extend or retract
     */
    @Override
    public void initialize() {
        if (m_intake.getIntakePistonExtendStatus() != extend)
        m_intake.setIntakePiston(extend);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
    }
