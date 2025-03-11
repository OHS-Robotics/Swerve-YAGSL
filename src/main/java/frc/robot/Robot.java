// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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
    
    SmartDashboard.putNumber("Front Left - Drive Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[0].getDriveMotor().getPosition());
    SmartDashboard.putNumber("Front Left - Turn Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[0].getAngleMotor().getPosition());
    SmartDashboard.putNumber("Front Left - Abs Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[0].getAbsoluteEncoder().getAbsolutePosition());

    SmartDashboard.putNumber("Front Right - Drive Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[1].getDriveMotor().getPosition());
    SmartDashboard.putNumber("Front Right - Turn Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[1].getAngleMotor().getPosition());
    SmartDashboard.putNumber("Front Right - Abs Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[1].getAbsoluteEncoder().getAbsolutePosition());

    SmartDashboard.putNumber("Back Left - Drive Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[2].getDriveMotor().getPosition());
    SmartDashboard.putNumber("Back Left - Turn Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[2].getAngleMotor().getPosition());
    SmartDashboard.putNumber("Back Left - Abs Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[2].getAbsoluteEncoder().getAbsolutePosition());

    SmartDashboard.putNumber("Back Right - Drive Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[3].getDriveMotor().getPosition());
    SmartDashboard.putNumber("Back Right - Turn Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[3].getAngleMotor().getPosition());
    SmartDashboard.putNumber("Back Right - Abs Pos", m_robotContainer.drivebase.getSwerveDrive().getModules()[3].getAbsoluteEncoder().getAbsolutePosition());
  
    SmartDashboard.putData("Field", m_robotContainer.drivebase.swerveDrive.field);
    // SmartDashboard.putData(SendableCameraWrapper.wrap("limelight", "http://limelight.local:5800/stream.mjpg"));

    // SmartDashboard.putNumber("Orientation", m_robotContainer.drivebase.getPose().getRotation().getDegrees());

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
    m_robotContainer.elevator.update();
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
