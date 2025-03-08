package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJogUp extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorJogUp(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.jogUp(Constants.Elevator.jogUpVel_InchesPerSec);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
