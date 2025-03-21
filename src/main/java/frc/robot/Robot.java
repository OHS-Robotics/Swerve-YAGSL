// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public Robot() {
    m_robotContainer = new RobotContainer();
    enableLiveWindowInTest(true);
  }

  @Override
  public void robotPeriodic() {
    // System.out.println(m_robotContainer.driverJoystick.getTwist());
    CommandScheduler.getInstance().run();
  
    SmartDashboard.putData("Field", m_robotContainer.drivebase.swerveDrive.field);

    SmartDashboard.putNumber("Swerve Pose (X, m)", m_robotContainer.drivebase.swerveDrive.getPose().getX());
    SmartDashboard.putNumber("Swerve Pose (Y, m)", m_robotContainer.drivebase.swerveDrive.getPose().getY());
    SmartDashboard.putNumber("Swerve Pose (R, deg)", m_robotContainer.drivebase.swerveDrive.getPose().getRotation().getDegrees());

    // SmartDashboard.putData(SendableCameraWrapper.wrap("limelight", "http://limelight.local:5800/stream.mjpg"));
    // Shuffleboard.getTab("camera").add(SendableCameraWrapper.wrap("limelight", "http://limelight.local:5800/stream.mjpg"));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // disabled for competition
    m_autonomousCommand = m_robotContainer.getAutoInitCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.teleopInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.elevator.updateSmartDashboard();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void testExit() {}
}
