// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

// import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

	final CommandXboxController driverXbox = new CommandXboxController(0);
	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
	private boolean referenceFrameIsField = true;

	// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
	SwerveInputStream driveFieldAngularVelocityStream = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1, () -> driverXbox.getLeftX() * -1)
		.withControllerRotationAxis(driverXbox::getRightX)
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(OperatorConstants.JOYSTICK_SCALE_FACTOR)
		.allianceRelativeControl(true);
	
	// Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
	SwerveInputStream driveFieldDirectAngleStream = driveFieldAngularVelocityStream.copy()
		.withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
		.headingWhile(true);

	// Clone's the angular velocity input stream and converts it to a robotRelative input stream.
	SwerveInputStream driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
		.robotRelative(true)
		.allianceRelativeControl(false);
	
	SwerveInputStream driveFieldAngularVelocityKeyboardStream = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX())
		.withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(OperatorConstants.JOYSTICK_SCALE_FACTOR)	
		.allianceRelativeControl(true);

	SwerveInputStream driveRobotAngularVelocityKeyboardStream = driveFieldAngularVelocityStream.copy()
		.withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
		.robotRelative(true)
		.allianceRelativeControl(false);

	// Derive the heading axis with math!
	SwerveInputStream driveFieldDirectAngleKeyboardStream = driveFieldAngularVelocityKeyboardStream.copy()
		.withControllerHeadingAxis(
			() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
			() -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)).headingWhile(true);


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
		driverXbox.rightBumper().onTrue(Commands.runOnce(() -> { referenceFrameIsField = false; System.out.println("robot"); }));
		driverXbox.rightBumper().onFalse(Commands.runOnce(() -> { referenceFrameIsField = true; System.out.println("field"); }));
		// driverXbox.rightTrigger().onTrue(Commands.runOnce(() -> System.out.println("here")));

		// driverXbox.a().whileTrue(Commands.run(() -> DriveRobot(false), drivebase));
		// driverXbox.a().whileFalse(Commands.run(() -> DriveRobot(true), drivebase));

		if (RobotBase.isSimulation()) {
			//Reset the robot to a semi-arbritary position
			driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
		} 

		if (DriverStation.isTest()) {
			driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
			driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			driverXbox.back().whileTrue(drivebase.centerModulesCommand());
			driverXbox.leftBumper().onTrue(Commands.none());
			driverXbox.rightBumper().onTrue(Commands.none());
		}
		else {
			// driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			// driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
			// driverXbox.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
			driverXbox.start().whileTrue(Commands.none());
			driverXbox.back().whileTrue(Commands.none());
			driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			driverXbox.rightBumper().onTrue(Commands.none());
		}
	}


	/**
	 * Execute the swerve drive's drive function.
	 * Automatically selects the appropriate control scheme (keyboard for simulation) and reference frame (field vs. robot)
	*/
	public void DriveRobot(boolean referenceFrameIsField) {
		if (referenceFrameIsField) {
			if (Robot.isSimulation()) {
				drivebase.driveFieldOrientedImmediate(driveFieldAngularVelocityKeyboardStream);
			}
			else {
				drivebase.driveFieldOrientedImmediate(driveFieldAngularVelocityStream);
			}
		}
		else {
			if (Robot.isSimulation()) {
				drivebase.driveFieldOrientedImmediate(driveRobotAngularVelocityKeyboardStream);
			}
			else {
				drivebase.driveFieldOrientedImmediate(driveRobotAngularVelocityStream);
			}
		}
		
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}
