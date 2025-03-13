// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

// import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.autonomous.AutonomousSubsystem;
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
import frc.robot.commands.swervedrive.Nudge;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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

	private boolean referenceFrameIsField = false;
	public double elevatorPosition = 0.0;
	public boolean isInHighGear = true;

	// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
	SwerveInputStream driveFieldAngularVelocityStream = SwerveInputStream.of(drivebase.getSwerveDrive(), 
		() -> applyExpoCurveTranslation(driverJoystick.getY()), 
		() -> applyExpoCurveTranslation(driverJoystick.getX()))
		.withControllerRotationAxis(() -> applyExpoCurveRotation(driverJoystick.getTwist()))
		.deadband(Constants.Operator.deadband)
		.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
		.scaleRotation(Constants.Operator.scaleRotationHighGear)
		.allianceRelativeControl(true);
	
	// Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
	SwerveInputStream driveFieldDirectAngleStream = driveFieldAngularVelocityStream.copy()
		.withControllerHeadingAxis(driverJoystick::getX, () -> 0.0)
		.headingWhile(true);

	// Clone's the angular velocity input stream and converts it to a robotRelative input stream.
	SwerveInputStream driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
		.robotRelative(true)
		.allianceRelativeControl(false);
	
	SwerveInputStream driveFieldAngularVelocityKeyboardStream = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -driverJoystick.getY(), () -> -driverJoystick.getX())
		.withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
		.deadband(Constants.Operator.deadband)
		.scaleTranslation(Constants.Operator.scaleTranslationHighGear)
		.scaleRotation(Constants.Operator.scaleRotationHighGear)
		.allianceRelativeControl(true);

	SwerveInputStream driveRobotAngularVelocityKeyboardStream = driveFieldAngularVelocityStream.copy()
		.withControllerRotationAxis(() -> driverJoystick.getRawAxis(2))
		.robotRelative(true)
		.allianceRelativeControl(false);

	// Derive the heading axis with math!
	SwerveInputStream driveFieldDirectAngleKeyboardStream = driveFieldAngularVelocityKeyboardStream.copy()
		.withControllerHeadingAxis(
			() -> Math.sin(driverJoystick.getRawAxis(2) * Math.PI) * (Math.PI * 2),
			() -> Math.cos(driverJoystick.getRawAxis(2) * Math.PI) * (Math.PI * 2)).headingWhile(true);


	Command driveFieldDirectAngle = drivebase.driveFieldOriented(driveFieldDirectAngleStream);
	Command driveFieldAnglularVelocity = drivebase.driveFieldOriented(driveFieldAngularVelocityStream);
	Command driveRobotAngularVelocity = drivebase.driveFieldOriented(driveRobotAngularVelocityStream);
	// Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
	Command driveFieldDirectAngleKeyboard = drivebase.driveFieldOriented(driveFieldDirectAngleKeyboardStream);
	Command driveFieldAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveFieldAngularVelocityKeyboardStream);
	Command driveRobotAngularVelocityKeyboard = drivebase.driveFieldOriented(driveRobotAngularVelocityKeyboardStream);
	// Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);


	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		//Delegate the actual calling of the swerve drive function to 
		drivebase.setDefaultCommand(Commands.run(() -> DriveRobot(referenceFrameIsField), drivebase));
		// driverJoystick.button(13).onTrue(Commands.runOnce(() -> { referenceFrameIsField = !referenceFrameIsField; SmartDashboard.putString("Reference Frame", referenceFrameIsField ? "Field" : "Robot"); }));
		driverJoystick.button(2).onTrue(Commands.runOnce(() -> SetGearMode(false)));
		driverJoystick.button(2).onFalse(Commands.runOnce(() -> SetGearMode(true)));

		SmartDashboard.putString("Gear Mode", isInHighGear ? "High" : "Low");

		driverJoystick.button(15).onTrue(autonomous.tweakToCoralCommand());

		SetupCoralManipulatorCommands();
		SetupElevatorCommands();
		SetupNudgeCommands();

		if (RobotBase.isSimulation()) {
			//Reset the robot to a semi-arbritary position
			// driverJoystick.button(14).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
		} 

		if (DriverStation.isTest()) {
			// removed cuz we don't really need it
			/*driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
			driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			driverXbox.back().whileTrue(drivebase.centerModulesCommand());
			driverXbox.leftBumper().onTrue(Commands.none());
			driverXbox.rightBumper().onTrue(Commands.none());*/
		}
		else {
			// driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			// driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
			// driverXbox.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
			// driverJoystick.button(8).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
		}
	}

	private void SetupCoralManipulatorCommands() {
		LoadOrStopCoral loadOrStopCoral = new LoadOrStopCoral(coralManipulator);
        UnloadCoral unloadCoral = new UnloadCoral(coralManipulator);
		UnloadCoralTwist unloadCoralTwist = new UnloadCoralTwist(coralManipulator);

		loadOrStopCoral.coralManipulator = coralManipulator;
		unloadCoral.coralManipulator = coralManipulator;
		unloadCoralTwist.coralManipulator = coralManipulator;

		driverJoystick.button(3).onTrue(loadOrStopCoral);
		driverJoystick.button(4).onTrue(unloadCoral);
		driverJoystick.button(1).onTrue(unloadCoralTwist);
	}

	private void SetupElevatorCommands() {
		// ElevatorJogUp jogUp = new ElevatorJogUp(elevator);
		// ElevatorJogDown jogDown = new ElevatorJogDown(elevator);
		// ElevatorStop stop = new ElevatorStop(elevator);
		ElevatorBottom bottom = new ElevatorBottom(elevator);
		ElevatorLevel1 L1 = new ElevatorLevel1(elevator);
		ElevatorLevel2 L2 = new ElevatorLevel2(elevator);
		ElevatorLevel3 L3 = new ElevatorLevel3(elevator);
		ElevatorLevel4 L4 = new ElevatorLevel4(elevator);

		// driverJoystick.povUp().onTrue(jogUp);
		// driverJoystick.povUp().onFalse(stop);
		// driverJoystick.povDown().onTrue(jogDown);
		// driverJoystick.povDown().onFalse(stop);
		driverJoystick.button(5).onTrue(bottom);
		driverJoystick.button(6).onTrue(L1);
		driverJoystick.button(7).onTrue(L2);
		driverJoystick.button(10).onTrue(L3);
		driverJoystick.button(9).onTrue(L4);
	}

	private void SetupNudgeCommands() {
		driverJoystick.povUp().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistForward_Meters, 180, Constants.Operator.nudgeSpeed_MetersPerSec)); //Forward
		driverJoystick.povDown().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistBack_Meters, 0, Constants.Operator.nudgeSpeed_MetersPerSec)); //Backward
		driverJoystick.povLeft().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistLeft_Meters, 270, Constants.Operator.nudgeSpeed_MetersPerSec)); //Left
		driverJoystick.povRight().onTrue(new Nudge(drivebase, Constants.Operator.nudgeDistRight_Meters, 90, Constants.Operator.nudgeSpeed_MetersPerSec)); //Right
	}

	public void updateElevator(double change) {
		// elevatorPosition += change;
		// // todo: make these actual values
		// elevatorPosition = Math.max(0.0, Math.min(300.0, elevatorPosition));

		// elevator.moveAbsolute(elevatorPosition);
	}

	public void teleopInit() {
		// loadCoralComposed.execute();
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

	public Command getAutonomousCommand() {

		if (Constants.Autonomous.autoEnabled) {
			String autoPathName = null;

			// Paths should be configured for the "BLUE" alliance ad they will be
			// automtically reversed for the "RED" alliance as configured...
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
				System.out.println("Autonomous Aliance: " + alliance);
            }

			autoPathName = "blankpath";
			// autoPathName = "StartCenter";
			// autoPathName = "StartLeft";
	
			return drivebase.getAutonomousCommand(autoPathName);	
		} else {
			return Commands.print("WARNING: AUTONOMOUS MODE IS DISABLED");
		}
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}

	public Command getAutoInitCommand() {
		return autonomous.getStartCommand();
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
			driveFieldDirectAngleStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveFieldDirectAngleStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
			driveRobotAngularVelocityStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveRobotAngularVelocityStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
			driveFieldAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveFieldAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
			driveRobotAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveRobotAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
			driveFieldDirectAngleKeyboardStream.scaleRotation(Constants.Operator.scaleRotationHighGear);
			driveFieldDirectAngleKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationHighGear);
		}
		else {
			driveFieldAngularVelocityStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveFieldAngularVelocityStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveFieldDirectAngleStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveFieldDirectAngleStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveRobotAngularVelocityStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveRobotAngularVelocityStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveFieldAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveFieldAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveRobotAngularVelocityKeyboardStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveRobotAngularVelocityKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
			driveFieldDirectAngleKeyboardStream.scaleRotation(Constants.Operator.scaleRotationLowGear);
			driveFieldDirectAngleKeyboardStream.scaleTranslation(Constants.Operator.scaleTranslationLowGear);
		}

		SmartDashboard.putString("Gear Mode", isInHighGear ? "High" : "Low");
	}
}
