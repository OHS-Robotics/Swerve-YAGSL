package frc.robot.subsystems.autonomous;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoReleaseCoralCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import frc.robot.subsystems.external.LimelightHelpers;
import frc.robot.subsystems.external.LimelightHelpers.RawFiducial;

/**
 * This is an example subsystem used in Autonomous mode (kinda).
 * Most of the work actually done by the "PathPlanner" tool/library.
 * 
 * For using PathPlanner:
 *   0) This is based on: "FRC Autonomous tutorial w/YAGSL Part 4 (2025)"
 *        - https://www.youtube.com/watch?v=mxZeFPbrNbE
 *   1) Install "FRC Pathplanner" application:
 *        - Installer:  https://apps.microsoft.com/detail/9NQBKB5DW909?hl=en-us&gl=US&ocid=pdpshare
 *        - Doc: https://github.com/mjansen4857/pathplanner/releases
 *   2) Open PathPlanner
 *        - You will be prompted to provide the root folder of the Robot Repository.
 *        - It needs this to save path configuration in the Robot Project.
 *        - This is the repository folder cloned on you computer, so it's location may vary
 *        - Example:  C:\OronoRobotics\Swerve-YAGSL
 *   3) There are PathPlanner settings to configure the attributes of the robot configuration.
 *        - Many of these might be the same as the setting plugged into the Yagsl Swerve Configuration.
 *   4) Define "Paths" and "Auto" items in Path Planner.
 *        - See Hudson for more information.
 *        - PATHS SHOULD BE DEFINED FOR THE BLUE ALLIANCE (AUTOMATICALLY REVERSED FOR RED ALLIANCE AS NEEDED?)
 *        - A "Path" is an instruction to move the Robot. 
 *        - "Path"s become segments of composite "Auto" paths.
 *        - "Auto" paths are made of "Path"s and other things like "Named Commands".
 *        - "Auto" Paths are used in the Robot code to perform during the Automomous period.
 *   5) Some code was added to SwerveSubsystem to make this work:
 *        - See: "setupPathPlanner()"  [called and defined...]
 *        - See: "getAutonomousCommand"
 *        - TO TEST: There are comments indicating "BLUE" paths are switched for "RED" alliance automatically.
 *   6) The Autonomous mode "Auto" name is used in the RobotContainer.java
 *        - If you create an "Auto" named "StartRight", you must load it with:
 *        -  String autoPathName = "StartRight";
 *        -  return drivebase.getAutonomousCommand(autoPathName);
 *   7) The "Path" and "Auto" instructions are built/stored in: ...\src\main\deploy\pathplanner\
 *   8) You can use Command Based classes from the Robot Project in the PathPlanner "Auto" definitions.
 *        - See the "AutoReleaseCoralCommand"...
 *        - The RobotContainer makes the Project command available to PathPlanner by registering a "NamedCommand"
 *        - NamedCommands.registerCommand("releaseCoral", autonomousSubsystem.getReleaseCoralCommand());
 *        - The PathPlanner "Auto" runs the command when the robot has reached the Coral tree.
 */
public class AutonomousSubsystem extends SubsystemBase {
    public SwerveSubsystem swerveDrive;
    private final PathConstraints constraints = new PathConstraints(2., 2.0, 2*Math.PI, 4*Math.PI);
    private AprilTagFieldLayout field;

    public AutonomousSubsystem(SwerveSubsystem swerve) {
        swerveDrive = swerve;
        field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public Command getReleaseCoralCommand() {
        return new AutoReleaseCoralCommand();
    }

    
    public PathPlannerPath getStartPath() {
        return getPathTo(swerveDrive.getPose().plus(new Transform2d(0.0, 0.5, new Rotation2d(0.0))));
    }

    public Command getStartCommand() {
        return AutoBuilder.followPath(getStartPath());
    }

    public PathPlannerPath getPathTo(Pose2d pose) {
        // waypoints is a list of points on the field to go to, with a minimum of two entries
        // since we just want the bot to go to one point, the first one is the position of the bot and the second is the desired position
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(swerveDrive.getPose(), pose);
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, pose.getRotation()));
        path.preventFlipping = true;

        return path;
    }

    public Command tweakToCoralCommand() {
        RawFiducial target = new RawFiducial(0, 0, 0, 0, 0, 0, 0);
        double dist = 9999999.9; // arbitrarily large value that will be overwritten if a tag is found
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        
        // find the nearest april tag because that's probably the one we're trying to move to
        if (fiducials.length < 1) {
            System.out.println("WARNING: No april tags found to adjust to!");
            return Commands.none();
        }
        for (RawFiducial fiducial : fiducials) {
            if (fiducial.distToRobot < dist) {
                dist = fiducial.distToRobot;
                target = fiducial;
            }
        }
        Pose3d aprilTagPose = field.getTagPose(target.id).get();
        double aprilTagRotation = aprilTagPose.getRotation().getZ();
        // this is kind of ugly but it calculates a vector in the direction the april tag is facing, it's a pose2d so we can use Pose2d.rotateBy()
        Pose2d targetOffset = new Pose2d(0.0, 1.0, new Rotation2d()).rotateBy(new Rotation2d(aprilTagRotation));
        // we convert this pose to a transform(and set the rotation to 180 so the pose is facing towards the april tag) and offset the april tag's position by this amount
        // we do this so we don't have the bot try to run into a wall
        Pose2d targetPose = aprilTagPose.toPose2d().plus(new Transform2d(targetOffset.getX(), targetOffset.getY(), new Rotation2d(Math.PI)));

        return AutoBuilder.followPath(getPathTo(targetPose));
    }
}
