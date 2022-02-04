package frc.robot.commands.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Controls;

public class OverrideAllianceColor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Controls m_controls;

  private final DriverStation.Alliance m_alliance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OverrideAllianceColor(Controls controls, DriverStation.Alliance alliance) {
    m_controls = controls;
    m_alliance = alliance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_controls);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled. Spins the Indexer and the
   * Kicker forward
   */
  @Override
  public void execute() {
    m_controls.setOverrideFmsAllianceColor(m_alliance);
  }

  /** Called once the command ends or is interrupted. Sets the Indexer and Kicker to a speed of 0 */
  @Override
  public void end(boolean interrupted) {
    m_controls.setOverrideFmsAlliance(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
