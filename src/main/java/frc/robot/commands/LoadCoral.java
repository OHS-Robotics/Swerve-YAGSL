package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class LoadCoral extends Command{
    public CoralManipulatorSubsystem coralLoader;

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!coralLoader.isIngestingCoral()) {
            coralLoader.ingestCoral();
        }
    }

    @Override
    public boolean isFinished() {
        return coralLoader.CoralLoaded();
    }
}
