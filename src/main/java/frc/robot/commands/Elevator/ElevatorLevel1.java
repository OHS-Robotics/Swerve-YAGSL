package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevel1 extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorLevel1(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        if (elevator.currentPosition_Inches() < Constants.Elevator.heightL1_Inches) {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightL1_Inches,  Constants.Elevator.moveUpVel_InchesPerSec);
        }
        else {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightL1_Inches,  Constants.Elevator.moveDownVel_InchesPerSec);
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
