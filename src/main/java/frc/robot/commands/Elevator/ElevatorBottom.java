package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorBottom extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorBottom(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        if (elevator.currentPosition_Revs() < Constants.Elevator.heightBottom_Inches) {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightBottom_Inches,  Constants.Elevator.moveUpVel_InchesPerSec);
        }
        else {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightBottom_Inches,  Constants.Elevator.moveDownVel_InchesPerSec);
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
