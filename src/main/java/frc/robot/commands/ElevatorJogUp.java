package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJogUp extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorJogUp(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Jog Up");
    }

    @Override
    public void execute() {
        elevator.jogUp(0.2);
    }

    @Override
    public boolean isFinished() {
        // return elevator.isAtSpeed();
        return true;
    }
}
