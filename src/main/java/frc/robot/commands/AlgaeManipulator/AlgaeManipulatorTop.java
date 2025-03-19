package frc.robot.commands.AlgaeManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;

public class AlgaeManipulatorTop extends Command{
    public AlgaeManipulatorSubsystem algaeManipulator;

    public AlgaeManipulatorTop(AlgaeManipulatorSubsystem algaeManipulator) {
        this.algaeManipulator = algaeManipulator;
    }

    @Override
    public void initialize() {
        if (algaeManipulator.currentPosition_Degrees() < Constants.AlgaeManipulator.positionTop_Degrees) {
            algaeManipulator.moveAbsoluteBegin(Constants.AlgaeManipulator.positionTop_Degrees,  Constants.AlgaeManipulator.speedRaise_DegPerSec);
        }
        else {
            algaeManipulator.moveAbsoluteBegin(Constants.AlgaeManipulator.positionTop_Degrees,  Constants.AlgaeManipulator.speedLower_DegPerSecond);
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
