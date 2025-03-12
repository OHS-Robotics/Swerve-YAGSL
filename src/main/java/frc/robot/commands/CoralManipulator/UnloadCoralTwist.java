package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class UnloadCoralTwist extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    public UnloadCoralTwist(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
        addRequirements(coralManipulator);
    }

    @Override
    public void initialize() {
        coralManipulator.expelCoralTwist();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (!coralManipulator.CoralLoaded()) {
            coralManipulator.stopMoving();
            return true;
        }
        return false;
    }
}
