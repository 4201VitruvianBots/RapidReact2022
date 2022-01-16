/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Retracts the climb motor
 */
public class RetractClimber extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;

    /**
     * Retracts the climb motor
     *
     * @param climber The climber used by this command.
     */
    public RetractClimber(final Climber climber) {
        this.m_climber = climber;
        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.m_climber.setClimbPiston(false);
        if (!this.m_climber.getClimbPistonExtendStatus())
            this.m_climber.setClimberPercentOutput(0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        this.m_climber.setClimberPercentOutput(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
