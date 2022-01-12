/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Extends the climb piston and motor
 */
public class ExtendClimber extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;
    private double timestamp;

    /**
     * Extends the climb piston and motor
     *
     * @param climber The climber used by this command.
     */
    public ExtendClimber(final Climber climber) {
        this.m_climber = climber;
        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //engage the piston
        this.m_climber.setClimbPiston(true);
        //wait a tiny bit of time before the next step
        this.timestamp = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if((Timer.getFPGATimestamp() - this.timestamp) < 0.2) {
            //rotate the motor counterclockwise to nick the ratchet
            this.m_climber.setClimberPercentOutput(- 0.25);
        } else if((Timer.getFPGATimestamp() - this.timestamp) < 0.5)
            this.m_climber.setClimberPercentOutput(0.5);
        else
            this.m_climber.setClimberPercentOutput(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        //stop the motor
        this.m_climber.setClimberPercentOutput(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - this.timestamp) > 0.5;
    }
}
