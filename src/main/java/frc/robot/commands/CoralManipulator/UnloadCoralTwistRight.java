package frc.robot.commands.CoralManipulator;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;

public class UnloadCoralTwistRight extends Command{
    public CoralManipulatorSubsystem coralManipulator;
    private int cycles;

    public UnloadCoralTwistRight(CoralManipulatorSubsystem manip) {
        coralManipulator = manip;
        addRequirements(coralManipulator);
    }

    @Override
    public void initialize() {
        coralManipulator.expelCoralTwistRight();
        cycles = 0;
    }

    @Override
    public void execute() {
        cycles++;
    }

    @Override
    public void end(boolean interrupted) {
        coralManipulator.stopMoving();
        System.out.println("Unload Coral Twist Right End (" + cycles + " cycles)");
        System.out.println(interrupted ? "Interrupted" : "Not Interrupted");
    }

    @Override
    public boolean isFinished() {
        return RobotBase.isReal() ? !coralManipulator.CoralLoaded() : true;
    }
}
