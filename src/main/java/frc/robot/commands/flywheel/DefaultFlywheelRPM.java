// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultFlywheelRPM extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Flywheel m_flywheel;
    /*private final Vision m_vision;*/
    private final boolean printed = false;
    private double time;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DefaultFlywheelRPM(Flywheel flywheel /* Vision vision*/) {
       m_flywheel = flywheel;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    /**
         * if vision gets valid target, then set shooter to 3000 RPM, else 0
         */
    @Override
    public void execute() {
        
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
