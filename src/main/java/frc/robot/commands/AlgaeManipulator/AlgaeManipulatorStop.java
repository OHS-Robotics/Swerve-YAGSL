package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;

public class AlgaeManipulatorStop extends Command{
    public AlgaeManipulatorSubsystem algaeManipulator;

    public AlgaeManipulatorStop(AlgaeManipulatorSubsystem algaeManipulator) {
        this.algaeManipulator = algaeManipulator;
        
    }

    @Override
    public void initialize() {
        algaeManipulator.stop();
        System.out.println("Algae Stop");
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
