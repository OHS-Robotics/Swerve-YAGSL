package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJog extends Command{
    public ElevatorSubsystem elevator;
    private CommandXboxController xbox;

    public ElevatorJog(ElevatorSubsystem elev, CommandXboxController c) {
        elevator = elev;
        xbox = c;
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Jog ");
    }

    @Override
    public void execute() {
        elevator.jogUp(xbox.getLeftY());
    }

    @Override
    public boolean isFinished() {
        // return elevator.isAtSpeed();
        return true;
    }

    private double applyDeadBand(double raw, double deadband) {
        if (Math.abs(raw) < deadband) return 0;
        
        if (raw - deadband > 0) {
            return (raw - deadband);
        }
        if (raw + deadband < 0) {
            return (raw + deadband);
        }
        
        return 0;
    }
}
