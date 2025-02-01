package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class StopCoral extends Command{
    public CoralManipulatorSubsystem coralManipulator;

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
