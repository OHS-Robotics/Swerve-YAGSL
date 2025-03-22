package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;

public class AlgaeManipulatorJogDown extends Command{
    public AlgaeManipulatorSubsystem algaeManipulator;

    public AlgaeManipulatorJogDown(AlgaeManipulatorSubsystem algaeManipulator) {
        this.algaeManipulator = algaeManipulator;
    }

    @Override
    public void initialize() {
        algaeManipulator.jogDown(Constants.AlgaeManipulator.speedLower_DegPerSec);
        System.out.println("Algae Jog Down");
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
