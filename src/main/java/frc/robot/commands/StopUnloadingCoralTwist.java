package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class StopUnloadingCoralTwist extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    public StopUnloadingCoralTwist(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralManipulator.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
