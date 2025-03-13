package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class LoadOrStopCoral extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    public LoadOrStopCoral(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
        addRequirements(coralManipulator);
    }

    @Override
    public void initialize() {
        if (coralManipulator.isIngestingCoral() || coralManipulator.isExpelingCoral()) {
            coralManipulator.stopMoving();
            this.cancel();
        }
        else {
            coralManipulator.ingestCoral();
        }
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
