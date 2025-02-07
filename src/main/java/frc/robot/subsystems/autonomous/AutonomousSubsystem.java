package frc.robot.subsystems.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoReleaseCoralCommand;

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

    public Command getReleaseCoralCommand() {
        return new AutoReleaseCoralCommand();
    }

}
