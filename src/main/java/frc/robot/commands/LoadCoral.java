package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class LoadCoral extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!coralManipulator.isIngestingCoral()) {
            coralManipulator.ingestCoral();
        }
    }

    @Override
    public boolean isFinished() {
        return coralManipulator.CoralLoaded();
    }
}
