package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class UnloadCoral extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (!coralManipulator.isExpellingCoral()) {
            coralManipulator.expellCoral();
        }
    }

    @Override
    public boolean isFinished() {
        return !coralManipulator.CoralLoaded();
    }
}
