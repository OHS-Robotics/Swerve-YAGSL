package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorBottom extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorBottom(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        if (elevator.currentPosition() < Constants.Elevator.heightBottom) {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightBottom,  Constants.Elevator.jogUpVelInchesPerSec);
        }
        else {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightBottom,  Constants.Elevator.jogDownVelInchesPerSec);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
