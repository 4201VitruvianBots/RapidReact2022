package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

public class InterruptingCommand extends CommandBase {
  private final Command m_interruptable;
  private final Command m_interrupt;
  private final BooleanSupplier m_condition;
  private boolean hasInterrupted = false;

  public InterruptingCommand(Command interruptable, Command interrupt, BooleanSupplier condition) {
    m_interruptable = interruptable;
    m_interrupt = interrupt;
    m_condition = condition;
  }

  @Override
  public void initialize() {
    m_interruptable.initialize();
  }

  @Override
  public void execute() {
    if (hasInterrupted) m_interrupt.execute();
    else m_interruptable.execute();

    if (m_condition.getAsBoolean()) {
      hasInterrupted = true;
      m_interruptable.cancel();
      m_interrupt.initialize();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (hasInterrupted) m_interrupt.end(interrupted);
    else m_interruptable.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return hasInterrupted ? m_interrupt.isFinished() : m_interruptable.isFinished(); 

  }
}
