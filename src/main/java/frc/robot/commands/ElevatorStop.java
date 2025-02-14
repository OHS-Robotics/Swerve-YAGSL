package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorStop extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorStop(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Stop");
    }

    @Override
    public void execute() {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
