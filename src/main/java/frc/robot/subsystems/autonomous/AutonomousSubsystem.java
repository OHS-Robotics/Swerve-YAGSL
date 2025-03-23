package frc.robot.subsystems.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;

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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.commands.CoralManipulator.AutoReleaseCoralCommand;
import frc.robot.commands.CoralManipulator.LoadOrStopCoral;
import frc.robot.commands.CoralManipulator.UnloadCoralTwist;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.swervedrive.Nudge;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
    // note: don't make different versions of the same path to change depending on the team
    // if you check "reset odometry" then it will be relative to whatever team you start on
    public SwerveSubsystem swerveDrive;
    public CoralManipulatorSubsystem coralManipulator;
    public ElevatorSubsystem elevatorSubsystem;
    private final PathConstraints constraints = new PathConstraints(1.0, 1.0, 2*Math.PI, 4*Math.PI);
    private AprilTagFieldLayout field;

     private class AprilTagController/* implements Sendable*/ {
        /*
        * 0 = go to april tag's position on the field using the position estimation
        * 1 = go to the april tag's position based on where it appears on the limelight
        * 2 = go to april tag's position based on where it appears on the limelight using the nudge command
        * anything else = ignore april tag related commands
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

    public AutonomousSubsystem(SwerveSubsystem swerve, ElevatorSubsystem elevator, CoralManipulatorSubsystem coral) {
        swerveDrive = swerve;
        coralManipulator = coral;
        elevatorSubsystem = elevator;
        field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        NamedCommands.registerCommand("ingestCoral", (new ElevatorBottom(elevatorSubsystem)).andThen(new LoadOrStopCoral(coralManipulator)));
        NamedCommands.registerCommand("elevatorL1", new ElevatorLevel1(elevatorSubsystem));
        NamedCommands.registerCommand("expelL1", (new ElevatorLevel1(elevatorSubsystem)).andThen(new UnloadCoralTwist(coralManipulator)));

        // placeholders, to be replaced when algae commands are finished
        NamedCommands.registerCommand("collectL2Algae", (new ElevatorLevel2(elevatorSubsystem).andThen(Commands.none())));
        NamedCommands.registerCommand("depositAlgaeProcessor", (new ElevatorBottom(elevatorSubsystem).andThen(Commands.none())));
        // v this is bad and we need to test the elevator height first!
        NamedCommands.registerCommand("depositAlgaeNet", (new ElevatorNet(elevatorSubsystem)).andThen(Commands.none()));

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

//      myAlliance = Alliance.Blue;  // Red|Blue
//      myDSLocation = 3;            // 1|2|3

        System.out.println("Autonomous Alliance: " + myAlliance);
        System.out.println("Drivers Station Location: " + myDSLocation);

        // The starting lines
        double redStartLine = 10;
        double blueStartLine = 7.5;

        // Starting positions on the starting lines
        double position_1 = 1.0;
        double position_2 = 4.0;
        double position_3 = 7.0;

        double speed_MPS = 0.75;

        // Middle path
        double length_middle_1 = 1.15;
        double length_middle_2 = 0.25;

        // Lengths of paths from the sides
        double length_side_1 = 1.40;
        double length_side_2 = 1.55;  // Towards the reef
        double length_side_2b = 1.20; // Back from the reef
        double length_side_3 = 4.00;

        // Initial for RED...
        double myStartLine = redStartLine;
        double myPosition1 = position_1;
        double myPosition2 = position_2;
        double myPosition3 = position_3;
        double myDS1Rotation = 60.0;
        double myDS2Rotation = 0.0;
        double myDS3Rotation = 300.0;

        // Blue overrides...
        if (myAlliance == Alliance.Blue) {
            myStartLine = blueStartLine;
            myPosition1 = position_3;
            myPosition2 = position_2;
            myPosition3 = position_1;
            myDS1Rotation = 240.0;
            myDS2Rotation = 180.0;
            myDS3Rotation = 120.0;
        }

        switch (myDSLocation) {
            case 1:
                Pose2d resetPose1 = new Pose2d(myStartLine, myPosition1, Rotation2d.fromDegrees(myDS1Rotation));
                swerveDrive.swerveDrive.resetOdometry(resetPose1);

                cmdGroup.addCommands(getCoralInjectCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_1, simConversion(180.0, myAlliance), speed_MPS));
                cmdGroup.addCommands(getElevatorUpCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_2, simConversion(240.0, myAlliance), speed_MPS));
                cmdGroup.addCommands(getCoralEjectCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_2b, simConversion(60.0, myAlliance), speed_MPS));
                cmdGroup.addCommands(getElevatorDownCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_3, simConversion(180.0, myAlliance), speed_MPS));
            break;

            case 3:
                Pose2d resetPose3 = new Pose2d(myStartLine, myPosition3, Rotation2d.fromDegrees(myDS3Rotation));
                swerveDrive.swerveDrive.resetOdometry(resetPose3);

                cmdGroup.addCommands(getCoralInjectCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_1, simConversion(180.0, myAlliance), speed_MPS));
                cmdGroup.addCommands(getElevatorUpCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_2, simConversion(120.0, myAlliance), speed_MPS));
                cmdGroup.addCommands(getCoralEjectCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_2b, simConversion(300.0,myAlliance), speed_MPS));
                cmdGroup.addCommands(getElevatorDownCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_side_3, simConversion(180.0, myAlliance), speed_MPS));
            break;

            default:
                Pose2d resetPose2 = new Pose2d(myStartLine, myPosition2, Rotation2d.fromDegrees(myDS2Rotation));
                swerveDrive.swerveDrive.resetOdometry(resetPose2);

                cmdGroup.addCommands(getCoralInjectCommands());
                cmdGroup.addCommands(getElevatorUpCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_middle_1, simConversion(180.0, myAlliance), speed_MPS));
                cmdGroup.addCommands(getCoralEjectCommands());
                cmdGroup.addCommands(new Nudge(swerveDrive, length_middle_2, simConversion(0.0, myAlliance), speed_MPS));
                cmdGroup.addCommands(getElevatorDownCommands());
            break;
        }

        return cmdGroup;
    }

    /**
     * The robot behaves differently in "simulation" vs. "real" mode...
     * For the "RED" alliance simulation, the direction is inversed 180 degrees.
     * For the "BLUE" alliance...
     * @param angle 360 degree notation
     * @param alliance
     * @return
     */
     private double simConversion(double angle, Alliance alliance) {
        if (alliance == Alliance.Red) {
            if (RobotBase.isReal()) {
                return angle;
            }
            return (angle + 180.0) % 360.0;
        } else {
            if (RobotBase.isReal()) {
                // TODO: TEST THIS...
                return (angle + 180.0) % 360.0;            
            }
            return angle;
        }
    }

    private Command[] getCoralInjectCommands() {
        ArrayList<Command> cmds = new ArrayList<>();
        cmds.add(Commands.print("***** INJECTING CORAL *****"));
        // cmds.add(new LoadOrStopCoral(coralManipulator));
        return cmds.toArray(new Command[0]);
    }

    private Command[] getCoralEjectCommands() {
        ArrayList<Command> cmds = new ArrayList<>();
        cmds.add(Commands.print("***** EJECTING CORAL *****"));
        // cmds.add(new UnloadCoralTwist(coralManipulator));
        return cmds.toArray(new Command[0]);
    }

    private Command[] getElevatorUpCommands() {
        ArrayList<Command> cmds = new ArrayList<>();
        cmds.add(Commands.print("***** Elevator Up *****"));
        // cmds.add(new ElevatorLevel1(elevatorSubsystem));
        return cmds.toArray(new Command[0]);
    }

    private Command[] getElevatorDownCommands() {
        ArrayList<Command> cmds = new ArrayList<>();
        cmds.add(Commands.print("***** Elevator Down *****"));
        // cmds.add(new ElevatorBottom(elevatorSubsystem));
        return cmds.toArray(new Command[0]);
    }

    public enum AutoCommandSource {
        Disabled,
        ManualNudge,
        PathPlannerAutoChooser
    }
}
