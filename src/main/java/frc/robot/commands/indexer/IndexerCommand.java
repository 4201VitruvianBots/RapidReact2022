/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;

/**
 * An example command that uses an example subsystem.
 */
public class IndexerCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    int tripped = 0;
    double setpoint, startTime;

    public IndexerCommand(Indexer indexer) {
        m_indexer = indexer;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
//    setpoint = m_indexer.getPosition() * 7 / (1.25 * Math.PI) * 20;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_indexer.getIntakeSensor() && tripped == 0) {
            tripped = 1;
            startTime = Timer.getFPGATimestamp();
        }
        if(tripped == 1)
            CommandScheduler.getInstance().schedule(new IncrementIndexer(m_indexer));

        if(Timer.getFPGATimestamp() - startTime > 0.1 && tripped == 1) {
            tripped = 0;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
