package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class UnloadCoralTwist extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    public UnloadCoralTwist(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        coralManipulator.expellCoralTwist();
    }

    @Override
    public boolean isFinished() {
        return !coralManipulator.CoralLoaded();
    }
}
