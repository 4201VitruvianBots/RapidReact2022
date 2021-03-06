package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.subsystems.DriveTrain;

/** Sets the drivetrain to neutral (coast/brake) */
public class SetDriveTrainNeutralMode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final DriveTrainNeutralMode m_mode;

  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param driveTrain The driveTrain used by this command.
   * @param mode {@link DriveTrainNeutralMode}: COAST, BRAKE, or HALF_BRAKE.
   */
  public SetDriveTrainNeutralMode(DriveTrain driveTrain, DriveTrainNeutralMode mode) {
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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
