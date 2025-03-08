package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class LoadCoral extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    public LoadCoral(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
        addRequirements(coralManipulator);
    }

    @Override
    public void initialize() {
        coralManipulator.ingestCoral();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (coralManipulator.CoralLoaded()) {
            coralManipulator.stopMoving();
            return true;
        }
        return false;
    }
}
