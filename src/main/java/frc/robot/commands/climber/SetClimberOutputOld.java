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

import java.util.function.DoubleSupplier;

/**
 * Raises/lowers the climber based on joystick input
 */
public class SetClimberOutputOld extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;
    private final DoubleSupplier m_input;

    private boolean currentDirection = true;
    private boolean movable, switchDirection;
    private double timestamp;

    /**
     * Raises/lowers the climber based on joystick input
     *
     * @param climber The climber used by this command.
     * @param input   Supplier for controlling the motor
     */
    public SetClimberOutputOld(final Climber climber, final DoubleSupplier input) {
        this.m_climber = climber;
        this.m_input = input;
        // Use addRequirements() here to declare subsystem dependencies.
        this.addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        final double input = Math.abs(this.m_input.getAsDouble()) > 0.2 ? this.m_input.getAsDouble() : 0;
        final int direction = input > 0 ? 1 : input < 0 ? -1 : 0;
        if (this.m_climber.getClimbState()) {
            if (direction != 0) {
                this.timestamp = Timer.getFPGATimestamp();
                if (direction == 1 && !this.currentDirection) {
                    this.movable = false;
                    this.switchDirection = true;
                } else if (direction <= 0 && this.currentDirection) {
                    this.movable = false;
                    this.switchDirection = false;
                }
            }

            if (this.movable) {
                final double output = (this.m_climber.getClimberPosition() < -512) && (input < 0) ? 0 : input;
                this.m_climber.setClimberPercentOutput(output);
            } else {
                if (this.switchDirection) {
                    this.climberReleaseSequence();
                } else
                    this.climberRetractSequence();
            }
        }
    }

    private void climberReleaseSequence() {
        this.m_climber.setClimbPiston(true);

        if (Math.abs(Timer.getFPGATimestamp() - this.timestamp) < 0.2)
            this.m_climber.setClimberPercentOutput(-0.25);
        else if (Math.abs(Timer.getFPGATimestamp() - this.timestamp) < 0.4)
            this.m_climber.setClimberPercentOutput(0.25);
        else {
            this.m_climber.setClimberPercentOutput(0);
            this.movable = true;
            this.currentDirection = true;
        }
    }

    private void climberRetractSequence() {
        this.m_climber.setClimbPiston(false);
        if (Math.abs(Timer.getFPGATimestamp() - this.timestamp) < 0.2)
            this.m_climber.setClimberPercentOutput(-0.25);
        else {
            this.m_climber.setClimberPercentOutput(0);
            this.movable = true;
            this.currentDirection = false;
        }
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
