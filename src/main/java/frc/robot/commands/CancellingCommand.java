package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

public class CancellingCommand extends CommandBase {
  private final Command m_command;
  private final BooleanSupplier m_condition;

  public CancellingCommand(Command command, BooleanSupplier condition) {
    m_command = command;
    m_condition = condition;
  }

  @Override
  public void initialize() {
    m_command.initialize();
  }

  @Override
  public void execute() {
    m_command.execute();
    if (m_condition.getAsBoolean()) {
      m_command.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
}
