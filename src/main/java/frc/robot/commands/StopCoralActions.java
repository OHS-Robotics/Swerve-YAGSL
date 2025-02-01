package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class StopCoralActions extends Command{
    public CoralManipulatorSubsystem coralLoader;

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralLoader.stopActions();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
