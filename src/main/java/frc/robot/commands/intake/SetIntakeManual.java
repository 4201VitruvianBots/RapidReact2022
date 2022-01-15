// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetIntakeManual extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final Indexer m_indexer;

    /**
     * @param intake The intake used by this command
     * @param indexer The indexer used by this command
     */
    public SetIntakeManual(Intake intake, Indexer indexer) {
        m_intake = intake;
        m_indexer = indexer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexerOutput(1);
        m_intake.setIntakePercentOutput(0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.setIndexerOutput(0);
        m_intake.setIntakePercentOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
