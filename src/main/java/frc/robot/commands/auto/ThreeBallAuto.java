// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Scores 3 cargo in the high goal and exits the tarmac */
public class ThreeBallAuto extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveTrain m_driveTrain;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ThreeBallAuto(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }
}
