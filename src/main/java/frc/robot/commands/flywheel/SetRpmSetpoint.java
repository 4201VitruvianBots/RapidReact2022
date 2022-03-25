/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class SetRpmSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Flywheel m_flywheel;

  private final Vision m_vision;
  private final DoubleSupplier m_RPM;

  /** Creates a new ExampleCommand. */
  public SetRpmSetpoint(Flywheel flywheel, Vision vision, DoubleSupplier RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_flywheel = flywheel;
    m_RPM = RPM;
    m_vision = vision;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("RpmSetpoint is running");
    m_vision.setLimelightLEDState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setRPM(m_RPM.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
