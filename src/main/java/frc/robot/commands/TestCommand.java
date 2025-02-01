package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class TestCommand extends Command{
    private int x = 0;

    @Override
    public void execute() {
        System.out.println("x: " + x);
        x++;
    }

    @Override
    public boolean isFinished() {
        return x > 100;
    }
}
