package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Nudge extends Command{
    public SwerveSubsystem swerve;
    private double duration_Seconds;
    private final Timer timer = new Timer();
    double velX;
    double velY;
    double targetAngle_Degrees;

    /**
     * Creates a command to move the swerve drive at a certain angle, speed, and distance
     * @param swerve The swerve drive to control
     * @param distance_Meters The distance to move, in meters
     * @param angle_Degrees The angle at which to move, in degrees (0 = back, 90 = right, 180 = forward, 270 = left)
     * @param speed_MetersPerSecond The speed at which to move, in m/s
     */
    public Nudge(SwerveSubsystem swerve, double distance_Meters, double angle_Degrees, double speed_MetersPerSecond) {
        this.swerve = swerve;
        targetAngle_Degrees = (angle_Degrees + 360) % 180;
        duration_Seconds = distance_Meters * 1.2 / speed_MetersPerSecond;
        velX = speed_MetersPerSecond * (Rotation2d.fromDegrees(angle_Degrees).minus(swerve.getPose().getRotation())).getCos();
        velY = speed_MetersPerSecond * (Rotation2d.fromDegrees(angle_Degrees).minus(swerve.getPose().getRotation())).getSin();

        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        swerve.drive(new ChassisSpeeds(velX, velY, 0));

        //Once all wheels have reached their target angle, start the timer
        for (int i = 0; i < swerve.getSwerveDrive().getStates().length; i++) {
            var wheelAngle = swerve.getSwerveDrive().getStates()[i].angle.getDegrees();
            if (!valueIsWithinTolerance((wheelAngle + 180) % 180, targetAngle_Degrees, 1) && !!valueIsWithinTolerance(wheelAngle + 180, targetAngle_Degrees, 1)){
                return;
            }
        }

        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration_Seconds);
    }

    public boolean valueIsWithinTolerance(double val, double target, double tolerance) {
        var absTarget = Math.abs(target);
        var absVal = Math.abs(val);
        return Math.abs(absTarget - absVal) < tolerance;
    }
}
