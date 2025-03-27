package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class UnloadCoralTwistLeft extends Command{
    public CoralManipulatorSubsystem coralManipulator;

    public UnloadCoralTwistLeft(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
        addRequirements(coralManipulator);
    }

    @Override
    public void initialize() {
        coralManipulator.expelCoralTwistLeft();
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
        return RobotBase.isReal() ? !coralManipulator.CoralLoaded() : true;
    }
}
