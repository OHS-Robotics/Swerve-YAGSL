package frc.robot.commands;

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
            elevator.moveAbsoluteStart(Constants.Elevator.heightBottom,  Constants.Elevator.jogUpVel);
        }
        else {
            elevator.moveAbsoluteStart(Constants.Elevator.heightBottom,  Constants.Elevator.jogDownVel);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (elevator.moveAbsoluteComplete()) {
            elevator.stop();
            return true;
        }

        return false;
    }
}
