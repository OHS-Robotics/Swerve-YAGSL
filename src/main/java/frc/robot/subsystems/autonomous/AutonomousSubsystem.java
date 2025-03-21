package frc.robot.subsystems.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.commands.CoralManipulator.AutoReleaseCoralCommand;
import frc.robot.commands.CoralManipulator.LoadOrStopCoral;
import frc.robot.commands.CoralManipulator.UnloadCoralTwist;
import frc.robot.commands.swervedrive.Nudge;
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
    public CoralManipulatorSubsystem coralManipulator;
    private final PathConstraints constraints = new PathConstraints(1.0, 1.0, 2*Math.PI, 4*Math.PI);
    private AprilTagFieldLayout field;

     private class AprilTagController/* implements Sendable*/ {
        /*
        * 0 = go to april tag's position on the field using the position estimation
        * 1 = go to the april tag's position based on where it appears on the limelight
        * 2 = ignore april tag related commands
        */
        private long aprilTagMethod = 2;
        
        @SuppressWarnings("unused")
        LongSupplier getAprilTagMethod = () -> aprilTagMethod;
        @SuppressWarnings("unused")
        LongConsumer setAprilTagMethod = (method) -> aprilTagMethod = method;

        
        
        /*@Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("AprilTagController");

            builder.addIntegerProperty("aprilTagMethod", getAprilTagMethod, setAprilTagMethod);

        }*/
    }
    
    public AprilTagController aprilTagController = new AprilTagController();

    public AutonomousSubsystem(SwerveSubsystem swerve, CoralManipulatorSubsystem coral) {
        coralManipulator = coral;
        swerveDrive = swerve;
        field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        NamedCommands.registerCommand("ingestCoral", new UnloadCoralTwist(coralManipulator));
        NamedCommands.registerCommand("expelCoral", new LoadOrStopCoral(coralManipulator));

        setupPathPlanner();
    }


    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner()
    {
        RobotConfig config; // Load the RobotConfig from the GUI settings. You should probably store this in your Constants file
        try
        {
            config = RobotConfig.fromGUISettings();
            final boolean enableFeedforward = true;

            // Configure AutoBuilder last
            AutoBuilder.configure(
                swerveDrive::getPose, // Robot pose supplier
                swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speedsRobotRelative, moduleFeedForwards) -> { // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    if (enableFeedforward)
                    {
                        swerveDrive.swerveDrive.drive(
                        speedsRobotRelative,
                        swerveDrive.swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                        moduleFeedForwards.linearForces());
                    }
                    else
                    {
                        swerveDrive.setChassisSpeeds(speedsRobotRelative);
                    }
                },
                new PPHolonomicDriveController(
                    new PIDConstants(Constants.Autonomous.kP, Constants.Autonomous.kI, Constants.Autonomous.kD), // Translation PID constants
                    new PIDConstants(Constants.Autonomous.kP, Constants.Autonomous.kI, Constants.Autonomous.kD) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent())
                    {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                swerveDrive // Reference to this subsystem to set requirements
            );
        } catch (Exception e)
        {
            // Handle exception as needed
            e.printStackTrace();
        }
    }


    public Command getReleaseCoralCommand() {
        return new AutoReleaseCoralCommand();
    }

    
    public PathPlannerPath getStartPath() {
        Pose2d offset = new Pose2d(0.0, 0.5, new Rotation2d());//.rotateBy(swerveDrive.swerveDrive.getYaw());
        return getPathTo(swerveDrive.getPose().plus(new Transform2d(offset.getX(), offset.getY(), new Rotation2d(0.0))));
        // return getPathInDirection(null)
    }

    public Command getStartCommand() {
        int method = 1;
        switch (method) {
            case 0: {
                return AutoBuilder.followPath(getStartPath());
            }
            case 1: {
                return new PathPlannerAuto("simpleauto"); // this moves forward a little bit
            }
            default: {
                return Commands.none();
            }
        }
        
    }

    // go to a pose
    public PathPlannerPath getPathTo(Pose2d pose) {
        // waypoints is a list of points on the field to go to, with a minimum of two entries
        // since we just want the bot to go to one point, the first one is the position of the bot and the second is the desired position
        // if we include the rotation here, the bot will try to move in that direction which isn't what we want
        // we correct for this by getting rid of the rotation part
        Transform2d sdRotationCorrection = new Transform2d(0.0, 0.0, swerveDrive.getPose().getRotation().times(-1.0));
        Transform2d poseRotationCorrection = new Transform2d(0.0, 0.0, pose.getRotation().times(-1.0));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(swerveDrive.getPose().plus(sdRotationCorrection), pose.plus(poseRotationCorrection));
        // we want to end with the robot not moving and facing in the desired direction 
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, pose.getRotation()));
        path.preventFlipping = true;

        return path;
    }

    // getPathTo, except this moves relative to a direction
    public PathPlannerPath getPathInDirection(Pose2d pose) {
        
        Transform2d sdRotationCorrection = new Transform2d(0.0, 0.0, swerveDrive.getPose().getRotation().times(-1.0));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(swerveDrive.getPose().plus(sdRotationCorrection), pose);
        // we want to end with the robot not moving and facing in the desired direction
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, pose.getRotation()));
        path.preventFlipping = true;

        return path;
    }

    public Command tweakToCoral() {
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
        // (int) aprilTagController.getAprilTagMethod.getAsLong()
        switch ((int) aprilTagController.aprilTagMethod) {
            case 0: {
                Optional<Pose3d> aprilTagPose = field.getTagPose(target.id);
                if (!aprilTagPose.isPresent()) {
                    System.out.println("WARNING: Couldn't find an april tag target!");
                    return Commands.none();
                }
                
                double aprilTagRotation = aprilTagPose.get().getRotation().getZ();
                // this is kind of ugly but it calculates a vector in the direction the april tag is facing, it's a pose2d so we can use Pose2d.rotateBy()
                Pose2d targetOffset = new Pose2d(0.0, 0.5588 /* measured as the perfect distance */, new Rotation2d()).rotateBy(new Rotation2d(aprilTagRotation));
                // we convert this pose to a transform(and set the rotation to 180 so the pose is facing towards the april tag) and offset the april tag's position by this amount
                // we do this so we don't have the bot try to run into a wall
                Pose2d targetPose = aprilTagPose.get().toPose2d().plus(new Transform2d(targetOffset.getX(), targetOffset.getY(), new Rotation2d(Math.PI)));
        
                System.out.println("Tweaking");
                return AutoBuilder.followPath(getPathTo(targetPose));
            }

            case 1: {
                // get the vector to the target and move in that direction
                // this won't adjust rotation because you can't get the direction of the april tag
                // from raw fiducials
                Pose2d targetOffset = new Pose2d(Math.sin(target.txnc) * target.distToRobot * 0.9, Math.cos(target.txnc) * target.distToRobot * 0.9, new Rotation2d()).rotateBy(swerveDrive.swerveDrive.getYaw());
                Pose2d targetPose = swerveDrive.getPose()
                .plus(new Transform2d(targetOffset.getX(), targetOffset.getY(), new Rotation2d())) // offset by vector
                .plus(new Transform2d(-0.4572, -0.2032, new Rotation2d())); // account for limelight offset


                System.out.println("Tweaking");
                return AutoBuilder.followPath(getPathTo(targetPose));
            }

            case 2: {
                return new Nudge(swerveDrive, target.distToRobot * 0.9, target.txnc + swerveDrive.swerveDrive.getYaw().getDegrees(), Constants.MAX_SPEED*0.5);
            }

            default: {
                return Commands.none();
            }
        }


    }

    public Command tweakToCoralCommand() {
        return Commands.run(() -> tweakToCoral().schedule(), swerveDrive);
    }

    public Command getBasicAutoCommand() {
        SequentialCommandGroup cmdGroup = new SequentialCommandGroup(new Command[0]);

        // Use the alliance and Driver Station Location to determine our path...
        Optional<Alliance> optAlliance = DriverStation.getAlliance();
        OptionalInt optDSLocation = DriverStation.getLocation();

        Alliance myAlliance = null;
        int myDSLocation = 0;

        if (optAlliance.isPresent()) {
            myAlliance = optAlliance.get();
        }

        if (optDSLocation.isPresent()) {
            myDSLocation = optDSLocation.getAsInt();
        }

        // myAlliance = Alliance.Red;  // Red|Blue
        // myDSLocation = 1;           // 1|2|3

        System.out.println("Autonomous Alliance: " + myAlliance);
        System.out.println("Drivers Station Location: " + myDSLocation);

        double speed_MPS = 0.75;

        // Lengths of paths from the sides
        double length_1 = 1.40;
        double length_2 = 1.65;
        double length_3 = 4.00;

        if (myAlliance == Alliance.Red) {
            switch (myDSLocation) {
                case 1:
                    swerveDrive.swerveDrive.resetOdometry(new Pose2d(10, 1, Rotation2d.fromDegrees(60)));
                    // Assumption: robot is positioned on proper side...
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_1, 0.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 60.0, speed_MPS));
                    cmdGroup.addCommands(getCoralCommands());
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 240.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_3, 0.0, speed_MPS));
                break;
    
                case 3:
                    swerveDrive.swerveDrive.resetOdometry(new Pose2d(10, 7, Rotation2d.fromDegrees(300)));
                    // Assumption: robot is positioned on proper side...
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_1, 0.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 300.0, speed_MPS));
                    cmdGroup.addCommands(getCoralCommands());
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 120.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_3, 0.0, speed_MPS));
                break;
    
                default:
                    swerveDrive.swerveDrive.resetOdometry(new Pose2d(10, 4, Rotation2d.fromDegrees(0)));
                    // Assumption: robot is positioned in middle...
                    cmdGroup.addCommands(new Nudge(swerveDrive, 1.20, 0, speed_MPS));
                    cmdGroup.addCommands(getCoralCommands());
                break;
            }
        }
        else {
            switch (myDSLocation) {
                case 1:
                    swerveDrive.swerveDrive.resetOdometry(new Pose2d(7.5, 1, Rotation2d.fromDegrees(120)));
                    // Assumption: robot is positioned on proper side...
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_1, 180.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 120.0, speed_MPS));
                    cmdGroup.addCommands(getCoralCommands());
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 300.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_3, 180.0, speed_MPS));
                break;
    
                case 3:
                    swerveDrive.swerveDrive.resetOdometry(new Pose2d(7.5, 7, Rotation2d.fromDegrees(240)));
                    // Assumption: robot is positioned on proper side...
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_1, 180.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 240.0, speed_MPS));
                    cmdGroup.addCommands(getCoralCommands());
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_2, 60.0, speed_MPS));
                    cmdGroup.addCommands(new Nudge(swerveDrive, length_3, 180.0, speed_MPS));
                break;
    
                default:
                    swerveDrive.swerveDrive.resetOdometry(new Pose2d(7.5, 4, Rotation2d.fromDegrees(180)));
                    // Assumption: robot is positioned in middle...
                    cmdGroup.addCommands(new Nudge(swerveDrive, 1.20, 180, speed_MPS));
                    cmdGroup.addCommands(getCoralCommands());
                break;
            }
        }

        return cmdGroup;
    }

    private Command[] getCoralCommands() {
        ArrayList<Command> cmds = new ArrayList<>();

        cmds.add(Commands.print("TODO: Coral Commands..."));
        // TODO: Add commands to:
        //   - position the elevator
        //   - eject the coral
        //   - etc...

        return cmds.toArray(new Command[0]);
    }

    public enum AutoCommandSource {
        Disabled,
        ManualNudge,
        PathPlannerAutoChooser
    }
}
