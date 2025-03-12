// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(10);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

  }

  public static class Operator
  {
    // Joystick Deadband
    public static final double deadband = 0.05;
    public static final double deadbandLeftY = 0.05;
    public static final double deadbandRightX = 0.05;
    public static final double turnConstant = 6;
    public static final double scaleTranslationHighGear = 0.8;
    public static final double scaleTranslationLowGear = 0.2;
    public static final double scaleRotationHighGear = 0.8;
    public static final double scaleRotationLowGear = 0.2;
    public static final boolean useKeyboardInSim = false; // Whether to expect keyboard or controller controls in sim
  }

  public static final class Autonomous {
    public static final boolean autoEnabled = false; // Perform Autonomous Mode [SAFETY SWITCH]
  }

  public static final class Elevator {
    public static final double heightBottom_Inches = 0;
    public static final double heightL1_Inches = 18.5;
    public static final double heightL2_Inches = 25.5;
    public static final double heightL3_Inches = 40.0;
    public static final double heightL4_Inches = 66.75;
    public static final double heightMax_Inches = 68;
    

    public static final double jogUpVel_InchesPerSec = 0.2; //0.1366;
    public static final double stopVel_InchesPerSec = 0.035; //Velocity which we actually apply to stop. Serves as holding torque
    public static final double jogDownVel_InchesPerSec = jogUpVel_InchesPerSec - stopVel_InchesPerSec; //0.0683;

    public static final double revsPerInch = 1.04;

    public static final double slewRate = 0.1;
    public static final double atVelocityToleranceRevs = 0.05;
  }

  public static final class CoralManipulator {
    public static final double coralSenseDistance_mm = 30; //Maximum threshold for sensing coral with the laser
  }

}