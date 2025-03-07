package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJogDown extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorJogDown(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Jog Down");
    }

    @Override
    public void execute() {
        elevator.jogDown(Constants.Elevator.jogDownVelInchesPerSec);
    }

    @Override
    public boolean isFinished() {
        // return elevator.isAtSpeed();
        return true;
    }
}
