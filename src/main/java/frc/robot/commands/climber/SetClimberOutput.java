/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Raises/lowers the climber based on joystick input
 */
public class SetClimberOutput extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;

    private final Joystick m_controller;

    /**
     * Creates a new SetClimberOutput.
     *
     * @param climber    The climber used by this command.
     * @param controller The joystick controller used by this command.
     */
    public SetClimberOutput(final Climber climber, final Joystick controller) {
        this.m_climber = climber;
        this.m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(climber);
    }

    private enum climberState {
        MOVING,
        STILL
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        final double input = Math.abs(this.m_controller.getRawAxis(5)) > 0.2 ? this.m_controller.getRawAxis(5) : 0;

        final climberState desiredDirection = input == 0 ? climberState.STILL : climberState.MOVING;

        switch (desiredDirection) {
            case MOVING:
                this.m_climber.disengagePistonBrake();
                this.m_climber.setClimberPercentOutput(input);
                break;
            case STILL:
                this.m_climber.engagePistonBrake();
                break;
            default:
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        this.m_climber.setClimberPercentOutput(0.0);
        this.m_climber.engagePistonBrake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
