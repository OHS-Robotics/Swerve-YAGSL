package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJogDown extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorJogDown(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.jogDown(0.1);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtSpeed();
    }
}
