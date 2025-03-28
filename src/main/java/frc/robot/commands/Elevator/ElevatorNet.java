package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorNet extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorNet(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        if (elevator.currentPosition_Inches() < Constants.Elevator.heightNet_Inches) {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightNet_Inches,  Constants.Elevator.moveUpVel_InchesPerSec);
        }
        else {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightNet_Inches,  Constants.Elevator.moveDownVel_InchesPerSec);
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
