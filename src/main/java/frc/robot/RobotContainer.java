// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.autonomous.AutonomousSubsystem;
import frc.robot.subsystems.autonomous.AutonomousSubsystem.AutoCommandSource;
import frc.robot.commands.AlgaeManipulator.AlgaeManipulatorBottom;
import frc.robot.commands.AlgaeManipulator.AlgaeManipulatorTop;
import frc.robot.commands.CoralManipulator.LoadOrStopCoral;
import frc.robot.commands.CoralManipulator.UnloadCoral;
import frc.robot.commands.CoralManipulator.UnloadCoralTwist;
import frc.robot.commands.Elevator.ElevatorBottom;
import frc.robot.commands.Elevator.ElevatorJogDown;
import frc.robot.commands.Elevator.ElevatorJogUp;
import frc.robot.commands.Elevator.ElevatorLevel1;
import frc.robot.commands.Elevator.ElevatorLevel2;
import frc.robot.commands.Elevator.ElevatorLevel3;
import frc.robot.commands.Elevator.ElevatorLevel4;
import frc.robot.commands.Elevator.ElevatorStop;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;


import swervelib.SwerveInputStream;

public class RobotContainer {
	// public final AutonomousSubsystem autonomousSubsystem = new AutonomousSubsystem();

	// final CommandXboxController driverXbox = new CommandXboxController(0);
	final CommandJoystick driverJoystick = new CommandJoystick(0);
	final CommandXboxController driverXbox = new CommandXboxController(1);

	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
	public final CoralManipulatorSubsystem coralManipulator = new CoralManipulatorSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();
	public final AutonomousSubsystem autonomous = new AutonomousSubsystem(drivebase);
	public final AlgaeManipulatorSubsystem algaeManipulator = new AlgaeManipulatorSubsystem();

	private final SendableChooser<Command> pathPlannerChooser;
	private final SendableChooser<AutoCommandSource> autoCommandSourceChooser;

	private boolean referenceFrameIsField = false;
	public double elevatorPosition = 0.0;
	public boolean isInHighGear = true;

	SwerveInputStream driveFieldAngularVelocityStream;
	SwerveInputStream driveRobotAngularVelocityStream;
	SwerveInputStream driveFieldAngularVelocityKeyboardStream;
	SwerveInputStream driveRobotAngularVelocityKeyboardStream;

	Command driveFieldAnglularVelocity;
	Command driveRobotAngularVelocity;
	Command driveFieldAnglularVelocityKeyboard;
	Command driveRobotAngularVelocityKeyboard;


	public RobotContainer() {
		pathPlannerChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("PathPlanner Chooser", pathPlannerChooser);

		autoCommandSourceChooser = new SendableChooser<AutoCommandSource>();
		autoCommandSourceChooser.addOption("Disabled", AutoCommandSource.Disabled);
		autoCommandSourceChooser.addOption("Manual Nudge", AutoCommandSource.ManualNudge);
		autoCommandSourceChooser.addOption("PathPlanner via AutoChooser", AutoCommandSource.PathPlannerAutoChooser);
		autoCommandSourceChooser.setDefaultOption("Disabled", AutoCommandSource.Disabled);
		SmartDashboard.putData("Auto Command Source", autoCommandSourceChooser);

		configureDriveInputStreams();
		configureBindings();
	}

	private void configureDriveInputStreams() {
		if (Constants.Operator.useJoystick) {
			// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
			driveFieldAngularVelocityStream = SwerveInputStream.of(
					drivebase.getSwerveDrive(), 
					() -> applyExpoCurveTranslation(driverJoystick.getY()), 
					() -> applyExpoCurveTranslation(driverJoystick.getX()))
				.withControllerRotationAxis(() -> applyExpoCurveRotation(driverJoystick.getTwist()))
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			// Clone's the angular velocity input stream and converts it to a robotRelative input stream.
			driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
				.robotRelative(true)
				.allianceRelativeControl(false);

			driveFieldAngularVelocityKeyboardStream = SwerveInputStream.of(
					drivebase.getSwerveDrive(), 
					() -> -driverJoystick.getY(), 
					() -> -driverJoystick.getX())
				.withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			driveRobotAngularVelocityKeyboardStream = driveFieldAngularVelocityStream.copy()
				.withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
				.robotRelative(true)
				.allianceRelativeControl(false);
		}
		else {
			// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
			driveFieldAngularVelocityStream = SwerveInputStream.of(
					drivebase.getSwerveDrive(), 
					() -> applyExpoCurveTranslation(driverXbox.getLeftY()), 
					() -> applyExpoCurveTranslation(driverXbox.getLeftX()))
				.withControllerRotationAxis(() -> applyExpoCurveRotation(driverXbox.getRightX()))
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			// Clone's the angular velocity input stream and converts it to a robotRelative input stream.
			driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
				.robotRelative(true)
				.allianceRelativeControl(false);

			driveFieldAngularVelocityKeyboardStream = SwerveInputStream.of(drivebase.getSwerveDrive(), 
					() -> -driverXbox.getLeftY(), 
					() -> -driverXbox.getLeftX())
				.withControllerRotationAxis(() -> driverXbox.getRightX())
				.deadband(Constants.Operator.deadband)
				.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
				.scaleRotation(Constants.Operator.scaleRotationHighGear)
				.allianceRelativeControl(true);

			driveRobotAngularVelocityKeyboardStream = driveFieldAngularVelocityStream.copy()
				.withControllerRotationAxis(() -> driverXbox.getRightX())
				.robotRelative(true)
				.allianceRelativeControl(false);
		}

		driveFieldAnglularVelocity = drivebase.driveFieldOriented(driveFieldAngularVelocityStream);
		driveRobotAngularVelocity = drivebase.driveFieldOriented(driveRobotAngularVelocityStream);
		driveFieldAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveFieldAngularVelocityKeyboardStream);
		driveRobotAngularVelocityKeyboard = drivebase.driveFieldOriented(driveRobotAngularVelocityKeyboardStream);		
	}

	private void configureBindings() {
		//Delegate the actual calling of the swerve drive function to DriveRobot()
		drivebase.setDefaultCommand(Commands.run(() -> DriveRobot(referenceFrameIsField), drivebase));

		SetupAutonomousCommands();
		SetupLowGearModeCommands();
		SetupCoralManipulatorCommands();
		SetupElevatorCommands();
		SetupNudgeCommands();
		SetupAlgaeManipulatorCommands();

		if (RobotBase.isSimulation()) {
			// Reset the robot to a semi-arbritary position
			// driverJoystick.button(14).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
		}
	}

	private void SetupAutonomousCommands() {
		// disabled for competition
		// driverJoystick.button(15).onTrue(autonomous.tweakToCoralCommand());
	}

	private void SetupAlgaeManipulatorCommands() {
		AlgaeManipulatorTop top = new AlgaeManipulatorTop(algaeManipulator);
		AlgaeManipulatorBottom bottom = new AlgaeManipulatorBottom(algaeManipulator);

		if (Constants.Operator.useJoystick) {
			driverJoystick.povLeft().onTrue(top);
			driverJoystick.povRight().onTrue(bottom);
		}
		else {
			driverXbox.povLeft().onTrue(top);
			driverXbox.povRight().onTrue(bottom);
		}
	}

	private void SetupLowGearModeCommands() {
		if (Constants.Operator.useJoystick) {
			driverJoystick.button(2).onTrue(Commands.runOnce(() -> SetGearMode(false)));
			driverJoystick.button(2).onFalse(Commands.runOnce(() -> SetGearMode(true)));
		}
		else {
			driverXbox.leftStick().onTrue(Commands.runOnce(() -> SetGearMode(false)));
			driverXbox.rightStick().onTrue(Commands.runOnce(() -> SetGearMode(false)));
			driverXbox.leftStick().onFalse(Commands.runOnce(() -> SetGearMode(true)));
			driverXbox.rightStick().onFalse(Commands.runOnce(() -> SetGearMode(true)));
		}

		SmartDashboard.putString("Gear Mode", isInHighGear ? "High" : "Low");
	}

	private void SetupCoralManipulatorCommands() {
		LoadOrStopCoral loadOrStopCoral = new LoadOrStopCoral(coralManipulator);
        UnloadCoral unloadCoral = new UnloadCoral(coralManipulator);
		UnloadCoralTwist unloadCoralTwist = new UnloadCoralTwist(coralManipulator);

		loadOrStopCoral.coralManipulator = coralManipulator;
		unloadCoral.coralManipulator = coralManipulator;
		unloadCoralTwist.coralManipulator = coralManipulator;

		if (Constants.Operator.useJoystick) {
			driverJoystick.button(3).onTrue(loadOrStopCoral);
			driverJoystick.button(4).onTrue(unloadCoral);
			driverJoystick.button(1).onTrue(unloadCoralTwist);
		}
		else {
			driverXbox.leftBumper().onTrue(loadOrStopCoral);
			driverXbox.rightBumper().onTrue(unloadCoral);
			driverXbox.rightTrigger().onTrue(unloadCoralTwist);
		}
		
	}

	private void SetupElevatorCommands() {
		
		ElevatorBottom bottom = new ElevatorBottom(elevator);
		ElevatorLevel1 L1 = new ElevatorLevel1(elevator);
		ElevatorLevel2 L2 = new ElevatorLevel2(elevator);
		ElevatorLevel3 L3 = new ElevatorLevel3(elevator);
		ElevatorLevel4 L4 = new ElevatorLevel4(elevator);

		ElevatorJogUp jogUp = new ElevatorJogUp(elevator);
		ElevatorJogDown jogDown = new ElevatorJogDown(elevator);
		ElevatorStop stop = new ElevatorStop(elevator);

		if (Constants.Operator.useJoystick) {
			driverJoystick.button(5).onTrue(bottom);
			driverJoystick.button(6).onTrue(L1);
			driverJoystick.button(7).onTrue(L2);
			driverJoystick.button(10).onTrue(L3);
			driverJoystick.button(9).onTrue(L4);

			driverJoystick.povUp().onTrue(jogUp);
			driverJoystick.povDown().onTrue(jogDown);
			driverJoystick.povCenter().onTrue(stop);
		}
		else {
			driverXbox.start().onTrue(bottom);
			driverXbox.a().onTrue(L1);
			driverXbox.b().onTrue(L2);
			driverXbox.x().onTrue(L3);
			driverXbox.y().onTrue(L4);

			driverXbox.povUp().onTrue(jogUp);
			driverXbox.povDown().onTrue(jogDown);
			driverXbox.povCenter().onTrue(stop);
		}
	}

	private void SetupNudgeCommands() {
		// if (Constants.Operator.useJoystick) {
		// 	driverJoystick.povLeft().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistLeft_Meters, 270, Constants.Operator.nudgeSpeed_MetersPerSec)); //Left
		// 	driverJoystick.povRight().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistRight_Meters, 90, Constants.Operator.nudgeSpeed_MetersPerSec)); //Right
		// }
		// else {
		// 	driverXbox.povLeft().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistLeft_Meters, 270, Constants.Operator.nudgeSpeed_MetersPerSec)); //Left
		// 	driverXbox.povRight().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistRight_Meters, 90, Constants.Operator.nudgeSpeed_MetersPerSec)); //Right
		// }
	}

	public void teleopInit() {
		
	}

	/**
	 * Execute the swerve drive's drive function.
	 * Automatically selects the appropriate control scheme (keyboard for simulation) and reference frame (field vs. robot)
	*/
	@SuppressWarnings("unused")
	public void DriveRobot(boolean referenceFrameIsField) {
		if (referenceFrameIsField) {
			if (Robot.isSimulation() && Constants.Operator.useKeyboardInSim) {
				
				drivebase.driveFieldOrientedImmediate(driveFieldAngularVelocityKeyboardStream);
			}
			else {
				drivebase.driveFieldOrientedImmediate(driveFieldAngularVelocityStream);
			}
		}
		else {
			if (Robot.isSimulation() && Constants.Operator.useKeyboardInSim) {
				drivebase.driveFieldOrientedImmediate(driveRobotAngularVelocityKeyboardStream);
			}
			else {
				drivebase.driveFieldOrientedImmediate(driveRobotAngularVelocityStream);
			}
		}
		
	}

	// public Command getAutonomousCommand() {
	// 	// currently unused
		
	// 	if ("PathFinder".equals(Constants.Autonomous.autoMode)) {
	// 		String autoPathName = null;

	// 		// Paths should be configured for the "BLUE" alliance ad they will be
	// 		// automtically reversed for the "RED" alliance as configured...
    //         var alliance = DriverStation.getAlliance();
    //         if (alliance.isPresent()) {
	// 			System.out.println("Autonomous Aliance: " + alliance);
    //         }

	// 		autoPathName = "simplepath";
	// 		// autoPathName = "StartCenter";
	// 		// autoPathName = "StartLeft";
	
	// 		return drivebase.getAutonomousCommand(autoPathName);	
	// 	} else {
	// 		return Commands.print("WARNING: AUTONOMOUS MODE IS DISABLED");
	// 	}
	// }

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	public Command getAutoInitCommand() {
		if (autoCommandSourceChooser.getSelected() == AutoCommandSource.PathPlannerAutoChooser) {
			return pathPlannerChooser.getSelected();
		} else if (autoCommandSourceChooser.getSelected() == AutoCommandSource.ManualNudge) {
			return autonomous.getBasicAutoCommand();
		} else {
			return Commands.print("WARNING: AUTONOMOUS MODE IS DISABLED");
		}
	}

	/**
	 * Transforms joystick input to give a desired smoothness for operation
	 * @param input
	 * @return
	 */
	public double applyExpoCurveTranslation(double input) {
		return input;
	}

	/**
	 * Transforms joystick input to give a desired smoothness for operation
	 * @param input
	 * @return
	 */
	public double applyExpoCurveRotation(double input) {
		return input;
	}

	/**
	 * Toggles between high and low gear mode
	 */
	public void toggleGearMode() {
		isInHighGear = !isInHighGear;
		SetGearMode(isInHighGear);		
	}

	/**
	 * Sets the gear mode for the robot
	 * @param setHigh The desired gear mode (true = high, low = false)
	 */
	public void SetGearMode(boolean setHigh) {
		isInHighGear = setHigh;

		if (isInHighGear) {
			driveFieldAngularVelocityStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveFieldAngularVelocityStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
			driveRobotAngularVelocityStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveRobotAngularVelocityStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
			driveFieldAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveFieldAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
			driveRobotAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveRobotAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
		}
		else {
			driveFieldAngularVelocityStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveFieldAngularVelocityStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveRobotAngularVelocityStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveRobotAngularVelocityStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveFieldAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveFieldAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveRobotAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveRobotAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
		}

		SmartDashboard.putString("Gear Mode", isInHighGear ? "High" : "Low");
	}
}