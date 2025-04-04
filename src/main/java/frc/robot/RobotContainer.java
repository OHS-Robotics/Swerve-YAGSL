// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.autonomous.AutonomousSubsystem;
import frc.robot.subsystems.autonomous.AutonomousSubsystem.AutoCommandSource;
import frc.robot.subsystems.autonomous.AutonomousSubsystem.Position;
import frc.robot.commands.AlgaeManipulator.AlgaeManipulatorJogDown;
import frc.robot.commands.AlgaeManipulator.AlgaeManipulatorJogUp;
import frc.robot.commands.AlgaeManipulator.AlgaeManipulatorStop;
import frc.robot.commands.CoralManipulator.ForceEject;
import frc.robot.commands.CoralManipulator.LoadOrStopCoral;
import frc.robot.commands.CoralManipulator.UnloadCoral;
import frc.robot.commands.CoralManipulator.UnloadCoralTwistLeft;
import frc.robot.commands.CoralManipulator.UnloadCoralTwistRight;
import frc.robot.commands.Elevator.ElevatorBottom;
import frc.robot.commands.Elevator.ElevatorJogDown;
import frc.robot.commands.Elevator.ElevatorJogUp;
import frc.robot.commands.Elevator.ElevatorLevel1;
import frc.robot.commands.Elevator.ElevatorLevel2;
import frc.robot.commands.Elevator.ElevatorLevel3;
import frc.robot.commands.Elevator.ElevatorLevel4;
import frc.robot.commands.Elevator.ElevatorSetZero;
import frc.robot.commands.Elevator.ElevatorStop;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;


import swervelib.SwerveInputStream;

public class RobotContainer {

	final CommandJoystick driverJoystick = new CommandJoystick(0);
	final CommandXboxController driverXbox = new CommandXboxController(0);
	final CommandGenericHID driverGenericHID = new CommandGenericHID(0);

	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
	public final CoralManipulatorSubsystem coralManipulator = new CoralManipulatorSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();
	public final AutonomousSubsystem autonomous = new AutonomousSubsystem(drivebase, elevator, coralManipulator);
	public final AlgaeManipulatorSubsystem algaeManipulator = new AlgaeManipulatorSubsystem();

	private final SendableChooser<Command> pathPlannerChooser;
	private final SendableChooser<AutoCommandSource> autoCommandSourceChooser;
	private final SendableChooser<Alliance> teamChooser;
	private final SendableChooser<Integer> positionChooser;

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

		teamChooser = new SendableChooser<Alliance>();
		teamChooser.addOption("Red", Alliance.Red);
		teamChooser.addOption("Blue", Alliance.Blue);
		SmartDashboard.putData("Auto Team", teamChooser);

		positionChooser = new SendableChooser<Integer>();
		for (int i = 1; i < 4; i++) {
			positionChooser.addOption("" + i, i);
		}
		positionChooser.setDefaultOption("Default(2)", 2);
		SmartDashboard.putData("Auto Position", positionChooser);

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
	}

	private void SetupAutonomousCommands() {
		// disabled for competition
		// driverJoystick.button(15).onTrue(autonomous.tweakToCoralCommand());
	}

	private void SetupAlgaeManipulatorCommands() {
		AlgaeManipulatorJogUp jogUp = new AlgaeManipulatorJogUp(algaeManipulator);
		AlgaeManipulatorJogDown jogDown = new AlgaeManipulatorJogDown(algaeManipulator);
		AlgaeManipulatorStop stop = new AlgaeManipulatorStop(algaeManipulator);

		if (Constants.Operator.useJoystick) {
			driverJoystick.povLeft().onTrue(jogUp);
			driverJoystick.povRight().onTrue(jogDown);
			driverJoystick.povCenter().onTrue(stop);
		}
		else {
			driverXbox.povLeft().onTrue(jogDown);
			driverXbox.povLeft().onFalse(stop);
			driverXbox.povRight().onTrue(jogUp);
			driverXbox.povRight().onFalse(stop);
		}
	}

	private void SetupLowGearModeCommands() {
		if (Constants.Operator.useJoystick) {
			driverJoystick.button(2).onTrue(Commands.runOnce(() -> SetGearMode(false)));
			driverJoystick.button(2).onFalse(Commands.runOnce(() -> SetGearMode(true)));
		}
		else {
			driverXbox.leftStick().onTrue(Commands.runOnce(() -> SetGearMode(false)));
			// driverXbox.rightStick().onTrue(Commands.runOnce(() -> SetGearMode(false)));
			driverXbox.leftStick().onFalse(Commands.runOnce(() -> SetGearMode(true)));
			// driverXbox.rightStick().onFalse(Commands.runOnce(() -> SetGearMode(true));
		}

		SmartDashboard.putString("Gear Mode", isInHighGear ? "High" : "Low");
	}

	private void SetupCoralManipulatorCommands() {
		LoadOrStopCoral loadOrStopCoral = new LoadOrStopCoral(coralManipulator);
        UnloadCoral unloadCoral = new UnloadCoral(coralManipulator);
		UnloadCoralTwistRight unloadCoralTwistRight = new UnloadCoralTwistRight(coralManipulator);
		UnloadCoralTwistLeft unloadCoralTwistLeft = new UnloadCoralTwistLeft(coralManipulator);
		ForceEject forceEject = new ForceEject(coralManipulator);

		loadOrStopCoral.coralManipulator = coralManipulator;
		unloadCoral.coralManipulator = coralManipulator;
		unloadCoralTwistRight.coralManipulator = coralManipulator;
		unloadCoralTwistLeft.coralManipulator = coralManipulator;

		if (Constants.Operator.useJoystick) {
			driverJoystick.button(3).onTrue(loadOrStopCoral);
			driverJoystick.button(4).onTrue(unloadCoral);
			driverJoystick.button(1).onTrue(unloadCoralTwistRight);
			//twist left not bound on joystick
		}
		else {
			driverXbox.leftBumper().onTrue(loadOrStopCoral);
			driverXbox.rightBumper().onTrue(unloadCoral);
			driverXbox.rightTrigger().onTrue(unloadCoralTwistRight);
			driverXbox.leftTrigger().onTrue(unloadCoralTwistLeft);
			driverXbox.rightStick().onTrue(forceEject);
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

		ElevatorSetZero zero = new ElevatorSetZero(elevator);

		if (Constants.Operator.useJoystick) {
			driverJoystick.button(5).onTrue(bottom);
			driverJoystick.button(6).onTrue(L1);
			driverJoystick.button(7).onTrue(L2);
			driverJoystick.button(10).onTrue(L3);
			driverJoystick.button(9).onTrue(L4);

			driverJoystick.povUp().onTrue(jogUp);
			driverJoystick.povDown().onTrue(jogDown);
			driverJoystick.povCenter().onTrue(stop);
			
			driverJoystick.button(15).onTrue(zero);
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

			driverXbox.back().onTrue(zero);
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

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	public Command getAutoInitCommand() {
		System.out.println(autoCommandSourceChooser.getSelected().toString());
		if (autoCommandSourceChooser.getSelected() == AutoCommandSource.PathPlannerAutoChooser) {
			return pathPlannerChooser.getSelected();
		} else if (autoCommandSourceChooser.getSelected() == AutoCommandSource.ManualNudge) {
			return autonomous.getBasicAutoCommand(teamChooser, positionChooser);
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
		double sign = input / Math.abs(input);
		return Math.pow(input, Constants.Operator.expoCurveExponentTranslation) * sign;
	}

	/**
	 * Transforms joystick input to give a desired smoothness for operation
	 * @param input
	 * @return
	 */
	public double applyExpoCurveRotation(double input) {
		double sign = input / Math.abs(input);
		return Math.pow(input, Constants.Operator.expoCurveExponentRotation) * sign;
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