package frc.robot.commands.swervedrive;

import java.util.Vector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class Nudge extends Command{
    public SwerveSubsystem swerve;
    private double duration_Seconds;
    private final Timer timer = new Timer();
    double velX;
    double velY;

    /**
     * Creates a command to move the swerve drive at a certain angle, speed, and distance
     * @param swerve The swerve drive to control
     * @param distance_Meters The distance to move, in meters
     * @param angle_Degrees The angle at which to move, in degrees (0 = back, 90 = right, 180 = forward, 270 = left)
     * @param speed_MetersPerSecond The speed at which to move, in m/s
     */
    public Nudge(SwerveSubsystem swerve, double distance_Meters, double angle_Degrees, double speed_MetersPerSecond) {
        this.swerve = swerve;
        duration_Seconds = distance_Meters / speed_MetersPerSecond;
        velX = speed_MetersPerSecond * (Rotation2d.fromDegrees(angle_Degrees).minus(swerve.getPose().getRotation())).getCos();
        velY = speed_MetersPerSecond * (Rotation2d.fromDegrees(angle_Degrees).minus(swerve.getPose().getRotation())).getSin();

        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        swerve.drive(new ChassisSpeeds(velX, velY, 0));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration_Seconds);
    }
}
