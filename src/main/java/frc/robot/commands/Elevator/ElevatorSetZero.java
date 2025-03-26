package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetZero extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorSetZero(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.zeroEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
