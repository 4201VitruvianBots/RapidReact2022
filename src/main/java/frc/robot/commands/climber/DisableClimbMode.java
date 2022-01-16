package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

/**
 * Disables control of the climber
 */
public class DisableClimbMode extends SequentialCommandGroup {
    /**
     * Disables control of the climber
     *
     * @param climber The climber used by this command.
     */
    public DisableClimbMode(Climber climber) {
        addCommands(
                new SetClimbMode(climber, false),
                new RetractClimber(climber));
    }

}
