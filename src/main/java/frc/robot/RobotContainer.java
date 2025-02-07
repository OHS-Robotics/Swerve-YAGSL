// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import java.util.function.DoubleSupplier;

// import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.autonomous.AutonomousSubsystem;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.UnloadCoral;
import frc.robot.commands.StopCoral;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import swervelib.SwerveInputStream;

public class RobotContainer {
	public final AutonomousSubsystem autonomousSubsystem = new AutonomousSubsystem();

	// final CommandXboxController driverXbox = new CommandXboxController(0);
	final CommandJoystick driverJoystick = new CommandJoystick(0);

	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
	public final CoralManipulatorSubsystem coralManipulator = new CoralManipulatorSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();

	private boolean referenceFrameIsField = true;
	private SequentialCommandGroup loadCoralComposed;
	public double elevatorPosition = 0.0;

	// Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
	SwerveInputStream driveFieldAngularVelocityStream = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> driverJoystick.getY() * -1, () -> driverJoystick.getX() * -1)
		.withControllerRotationAxis(() -> driverJoystick.getTwist() * -1)
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(OperatorConstants.JOYSTICK_SCALE_FACTOR)
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
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(OperatorConstants.JOYSTICK_SCALE_FACTOR)	
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

		// Named commands are used by PathPlanner for Autonomous Mode...
		NamedCommands.registerCommand("releaseCoral", autonomousSubsystem.getReleaseCoralCommand());
	}

	private void configureBindings() {
		//Delegate the actual calling of the swerve drive function to 
		drivebase.setDefaultCommand(Commands.run(() -> DriveRobot(referenceFrameIsField), drivebase));
		driverJoystick.button(13).onTrue(Commands.runOnce(() -> { referenceFrameIsField = !referenceFrameIsField; SmartDashboard.putString("Reference Frame", referenceFrameIsField ? "Field" : "Robot"); }));
		// driverJoystick.button(13).onFalse(Commands.runOnce(() -> { referenceFrameIsField = true; SmartDashboard.putString("Reference Frame", "Field");  }));
		// driverXbox.rightTrigger().onTrue(Commands.runOnce(() -> System.out.println("here")));
		// driverXbox.a().whileTrue(Commands.run(() -> DriveRobot(false), drivebase));
		// driverXbox.a().whileFalse(Commands.run(() -> DriveRobot(true), drivebase));

        LoadCoral loadCoralCommand = new LoadCoral();
        UnloadCoral unloadCoralCommand = new UnloadCoral();
		StopCoral stopCoralCommand = new StopCoral();

		driverJoystick.povUp().onTrue(Commands.runOnce(() -> updateElevator(0.05), elevator));
		driverJoystick.povDown().onTrue(Commands.runOnce(() -> updateElevator(-0.05), elevator));

        loadCoralCommand.addRequirements(coralManipulator);
		loadCoralCommand.coralManipulator = coralManipulator;

        unloadCoralCommand.addRequirements(coralManipulator);
		unloadCoralCommand.coralManipulator = coralManipulator;

		stopCoralCommand.addRequirements(coralManipulator);
		stopCoralCommand.coralManipulator = coralManipulator;

		loadCoralComposed = loadCoralCommand.andThen(stopCoralCommand);

		if (RobotBase.isSimulation()) {
			//Reset the robot to a semi-arbritary position
			driverJoystick.button(14).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
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
			driverJoystick.button(8).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
		}
	}

	public void updateElevator(double change) {
		elevatorPosition += change;
		// todo: make these actual values
		elevatorPosition = Math.max(0.0, Math.min(300.0, elevatorPosition));

		elevator.setPosition(elevatorPosition);
	}


	/**
	 * Execute the swerve drive's drive function.
	 * Automatically selects the appropriate control scheme (keyboard for simulation) and reference frame (field vs. robot)
	*/
	public void DriveRobot(boolean referenceFrameIsField) {		
		if (referenceFrameIsField) {
			if (Robot.isSimulation() && OperatorConstants.USE_KEYBOARD_IN_SIM) {
				drivebase.driveFieldOrientedImmediate(driveFieldAngularVelocityKeyboardStream);
			}
			else {
				drivebase.driveFieldOrientedImmediate(driveFieldAngularVelocityStream);
			}
		}
		else {
			if (Robot.isSimulation() && OperatorConstants.USE_KEYBOARD_IN_SIM) {
				drivebase.driveFieldOrientedImmediate(driveRobotAngularVelocityKeyboardStream);
			}
			else {
				drivebase.driveFieldOrientedImmediate(driveRobotAngularVelocityStream);
			}
		}
		
	}

	public Command getAutonomousCommand() {

		if (AutoConstants.AUTO_ENABLED) {
			String autoPathName = null;	

			// Paths should be configured for the "BLUE" alliance ad they will be
			// automtically reversed for the "RED" alliance as configured...
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
				System.out.println("Autonomous Aliance: " + alliance);
            }

			autoPathName = "StartRight";
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
}
