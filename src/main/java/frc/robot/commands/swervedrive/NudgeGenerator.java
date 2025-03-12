package frc.robot.commands.swervedrive;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public final class NudgeGenerator {

    public static Pose2d GenerateNudgeFowardPose(SwerveSubsystem swerve, double distance_Feet) {
        var currentPose = swerve.getPose();
        var translation = new Translation2d(
            currentPose.getX() + Units.feetToMeters(distance_Feet) * currentPose.getRotation().getCos(),
            currentPose.getY() + Units.feetToMeters(distance_Feet) * currentPose.getRotation().getSin());

        return  new Pose2d(translation, currentPose.getRotation());
    }

    public static final PathConstraints standardConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
}
