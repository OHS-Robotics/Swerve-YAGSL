package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class ForceEject extends Command{
    public CoralManipulatorSubsystem coralManipulator;
    public long startTime = 0;

    public ForceEject(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
        addRequirements(coralManipulator);
    }

    @Override
    public void initialize() {
        coralManipulator.expelCoral();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        coralManipulator.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > Constants.CoralManipulator.forceEjectDuration_ms;
    }
}
