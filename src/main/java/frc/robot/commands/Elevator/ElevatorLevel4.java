package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevel4 extends Command{
    public ElevatorSubsystem elevator;

    public ElevatorLevel4(ElevatorSubsystem elev) {
        elevator = elev;
    }

    @Override
    public void initialize() {
        if (elevator.currentPosition() < Constants.Elevator.heightL4) {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightL4,  Constants.Elevator.jogUpVelInchesPerSec);
        }
        else {
            elevator.moveAbsoluteBegin(Constants.Elevator.heightL4,  Constants.Elevator.jogDownVelInchesPerSec);
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
