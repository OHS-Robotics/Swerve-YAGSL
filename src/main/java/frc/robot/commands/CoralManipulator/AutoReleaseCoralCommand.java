package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command can be used for releasing Coral from the Arm/Appender when the robot is 
 * in autonomous mode and has moved to a position by the Reef Tree.
 */
public class AutoReleaseCoralCommand extends Command {

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {

    }

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public boolean isFinished() {
        // Only run once please!
        return true;
    }

}
