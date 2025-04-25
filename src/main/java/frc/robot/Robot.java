// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.kCameraName;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private PhotonCamera camera;

  private RobotContainer m_robotContainer = null;
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

    XboxController driverXbox = m_robotContainer.getXBoxController().getHID();

    // Calculate drivetrain commands from Joystick values
    double forward = -driverXbox.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
    double strafe = -driverXbox.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
    double turn = -driverXbox.getRightX() * Constants.Swerve.kMaxAngularSpeed;

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 7) {
                    // Found Tag 7, record its information
                    targetYaw = target.getYaw();
                    targetVisible = true;
                }
            }
        }
    }

    // Auto-align when requested
    if (driverXbox.getAButton() && targetVisible) {
        // Driver wants auto-alignment to tag 7
        // And, tag 7 is in sight, so we can turn toward it.
        // Override the driver's turn command with an automatic one that turns toward the tag.
        turn = -1.0 * targetYaw * kDefaultPeriod * Constants.Swerve.kMaxAngularSpeed;
    }

    // Command drivetrain motors based on target speeds
    m_robotContainer.drivebase.drive(forward, strafe, turn);

    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);

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
  
@Override
public void robotInit() {
        // drivetrain = new SwerveDrive();
        camera = new PhotonCamera(kCameraName); //CODA the camera object was declared above, but here is where it's actually initialized.  
                                                // You may need to change that kCameraName variable to match something you've set to it?
                                                // Pro Tip: click on a variable and right click -> go to definition to see whewre it's created (or press F12)                                  
        // visionSim = new VisionSim(camera);

            // Optional: Add an initial Shuffleboard entry
            Shuffleboard.getTab("Vision").addDouble("AprilTag ID", this::getAprilTagID);
        }
    
        /**
         * Get the AprilTag ID from the best target.
         */
        public double getAprilTagID() {
            PhotonPipelineResult result = camera.getLatestResult();
            if (result.hasTargets()) {
                return result.getBestTarget().getFiducialId();
            }
            return -1; // No tag found
        }
      }
