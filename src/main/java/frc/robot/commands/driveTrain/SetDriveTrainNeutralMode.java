package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

    /**
     * Sets the drivetrain to neutral (coast/brake)
     */
public class SetDriveTrainNeutralMode extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final int m_mode;

    /**
     * Sets the drivetrain to neutral (coast/brake)
     * 
     * @param driveTrain drivetrain to set
     * @param mode 2 = all coast, 1 = all brake, 0 = half and half
     */
    public SetDriveTrainNeutralMode(DriveTrain driveTrain, int mode) {
        m_driveTrain = driveTrain;
        m_mode = mode;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.setDriveTrainNeutralMode(m_mode);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
