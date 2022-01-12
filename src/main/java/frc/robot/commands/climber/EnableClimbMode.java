package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

/**
 * Enables control of the climber
 */
public class EnableClimbMode extends SequentialCommandGroup {
    /**
     * Enables control of the climber
     *
     * @param climber The climber used by this command.
     */
    public EnableClimbMode(final Climber climber) {
        addCommands(new SetClimbMode(climber, true), new ExtendClimber(climber));
    }

}
