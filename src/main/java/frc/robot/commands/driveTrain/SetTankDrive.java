/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

/**
 * Sets the left and right sides of the drivetrain separately based on joystick inputs
 */
public class SetTankDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final DoubleSupplier m_leftOutput, m_rightOutput;

    /**
     * Sets the left and right sides of the drivetrain separately based on joystick inputs
     * 
     * @param driveTrain drivetrain to set
     * @param leftOutput percent output for left side of drivetrain
     * @param rightOutput percent output for right side of drivetrain
     */
    public SetTankDrive(DriveTrain driveTrain, DoubleSupplier leftOutput, DoubleSupplier rightOutput) {
        m_driveTrain = driveTrain;
        m_leftOutput = leftOutput;
        m_rightOutput = rightOutput;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.setMotorTankDrive(m_leftOutput.getAsDouble(), m_rightOutput.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
