package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.requireUngrouped;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

/**
 * Runs a command until a condition is met, then interrupts it to run another commnd. If the first
 * command finishes, this command will end without running the other command.
 */
public class InterruptingCommand extends CommandBase {
  private final Command m_interruptible;
  private final Command m_interrupt;
  private Command m_selectedCommand;
  private final BooleanSupplier m_condition;
  private boolean hasInterrupted = false;

  /**
   * Runs a command until a condition is met, then interrupts it to run another commnd
   *
   * @param interruptible The command to run initially that will be interrupted
   * @param interrupt The command that interrupts the first command
   * @param condition When to interrupt the first command
   */
  public InterruptingCommand(Command interruptible, Command interrupt, BooleanSupplier condition) {
    requireUngrouped(interruptible, interrupt);

    //    CommandGroupBase.registerGroupedCommands(interruptible, interrupt);

    m_interruptible = interruptible;
    m_interrupt = interrupt;
    m_condition = condition;
    m_requirements.addAll(interruptible.getRequirements());
    m_requirements.addAll(interrupt.getRequirements());
  }

  @Override
  public void initialize() {
    hasInterrupted = false;
    m_selectedCommand = m_interruptible;
    m_selectedCommand.initialize();
  }

  @Override
  public void execute() {
    if (m_condition.getAsBoolean() && !hasInterrupted) {
      hasInterrupted = true;
      m_selectedCommand.end(true);
      m_selectedCommand = m_interrupt;
      m_selectedCommand.initialize();
    }
    m_selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }
}
