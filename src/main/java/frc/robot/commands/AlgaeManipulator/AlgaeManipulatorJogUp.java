package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;

public class AlgaeManipulatorJogUp extends Command{
    public AlgaeManipulatorSubsystem algaeManipulator;

    public AlgaeManipulatorJogUp(AlgaeManipulatorSubsystem algaeManipulator) {
        this.algaeManipulator = algaeManipulator;
    }

    @Override
    public void initialize() {
        algaeManipulator.jogUp(Constants.AlgaeManipulator.speedRaise_DegPerSec);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
