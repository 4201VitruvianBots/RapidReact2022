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
 * Sets climber mode
 */
//TODO: reconviene and decide if climbing should be done with a climb mode or done as one fluid command, one for going up motion and one for going down motion
public class SetClimbMode extends CommandBase {
    private final Climber m_climber;
    private final boolean m_mode;

    /**
     * Sets climber mode
     *
     * @param climber The subsystem used by this command.
     * @param mode    true is up, false is down.
     */
    public SetClimbMode(final Climber climber, final boolean mode) {
        this.m_climber = climber;
        this.m_mode = mode;
        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(climber);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.m_climber.setClimbState(this.m_mode);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
