package frc.robot.subsystems;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motor_left = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);

    // IMPORTANT NOTE: Right motor is configured as a follower of the left motor.  We will only command moves to the left motor
    
    SlewRateLimiter rateLimiterLeft = new SlewRateLimiter(Constants.Elevator.slewRate);
    SlewRateLimiter rateLimiterRight = new SlewRateLimiter(Constants.Elevator.slewRate);

    // left is the forward one, right is the actual one
    private final RelativeEncoder encoder_left = motor_left.getEncoder();
    private final RelativeEncoder encoder_right = motor_right.getEncoder();

    public double targetPosition_Revs = 0;
    private double targetVel_Revs = 0;
    private double targetVelCheckValue_Revs = 0;
    public boolean moveInProgress = false;
    public boolean accelerating = false; //true when the robot is not at target vel, meaning the slew rate limiter is doing work


    /**
     * Constructor for the elevator subsystem
     * @param revsPerInch how many rev's it takes to make the elevator move one inch
     */
    public ElevatorSubsystem() {
        final SparkMaxConfig baseconf_left = new SparkMaxConfig();
        final SparkMaxConfig baseconf_right = new SparkMaxConfig();

        encoder_left.setPosition(0);
        encoder_right.setPosition(0);

        baseconf_right.follow(motor_left, true);

        baseconf_left.smartCurrentLimit(30);
        baseconf_right.smartCurrentLimit(30);

        baseconf_left.idleMode(IdleMode.kBrake);
        baseconf_right.idleMode(IdleMode.kBrake);
        
        motor_left.configure(baseconf_left, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motor_right.configure(baseconf_right, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double currentPosition_Revs() {
        return -encoder_left.getPosition();
    }

    public double currentPosition_Inches() {
        return -encoder_left.getPosition() / Constants.Elevator.revsPerInch;
    }

    public double currentVelocity_RevsPerSec() {
        return -encoder_left.getVelocity();
    }

    public double currentVelocity_InchesPerSec() {
        return -encoder_left.getVelocity() / Constants.Elevator.revsPerInch;
    }
    
    public void update() {
        SmartDashboard.putNumber("Elevator Left", -encoder_left.getPosition());
        SmartDashboard.putNumber("Elevator Right", -encoder_right.getPosition());
    }

    /**
     * Begins a move to an absolute position.  The elevator will stop by itself when it reaches its target position
     * @param commandPosition Desired position (inches)
     */
    public void moveAbsoluteBegin(double position_Inches, double speed_InchesPerSecond) { // position is in inches
        targetPosition_Revs = position_Inches * Constants.Elevator.revsPerInch;
        targetVel_Revs = speed_InchesPerSecond * Constants.Elevator.revsPerInch;
        moveInProgress = true;

        if (targetPosition_Revs > currentPosition_Revs()) {
            jogUp(targetVel_Revs);
        }
        else {
            jogDown(targetVel_Revs);
        }

        System.out.println("Commanded Speed: " + targetVel_Revs);
    }

    public boolean atTargetPosition() {
        return valueIsWithinTolerance(currentPosition_Revs(), targetPosition_Revs, 0.5);
    }

    /**
     * Begins a move relative to the elevator's current position.  The elevator will stop by itself when it reaches its target distance
     * @param commandPosition Desired position (inches)
     */
    public void moveRelativeBegin(double distance_Inches, double speed_InchesPerSecond) { // commandDistance is in Inches
        var targetPositionInches = (distance_Inches * Constants.Elevator.revsPerInch + currentPosition_Revs()) / Constants.Elevator.revsPerInch;
        moveAbsoluteBegin(targetPositionInches, speed_InchesPerSecond);
    }


    /**
     * Jog the elevator up
     * @param speed_InchesPerSecond The speed at which you want to move, in inches/sec
     */
    public void jogUp(double speed_InchesPerSecond) {
        if (speed_InchesPerSecond < 0) {
            System.out.println("Cannot jog with a negative vel.  Provide only positive values");
            return;
        }

        //Up is negative for this elevator
        beginAcceleration(-speed_InchesPerSecond * Constants.Elevator.revsPerInch);
    }

    /**
     * Jog the elevator up
     * @param speed_InchesPerSecond The speed at which you want to move, in inches/sec
     */
    public void jogDown(double speed_InchesPerSecond) {
        if (speed_InchesPerSecond < 0) {
            System.out.println("Cannot jog with a negative vel.  Provide only positive values");
            return;
        }

        //Down is positive for this elevator
        beginAcceleration(speed_InchesPerSecond * Constants.Elevator.revsPerInch);
    }

    /**
     * Stops elevator motion
     */
    public void stop() {
        targetVel_Revs = -Constants.Elevator.stopVel_InchesPerSec;
        beginAcceleration(targetVel_Revs, 0);
    }

    /**
     * Begin accelerating to a speed
     * @param targetVel The velocity to which we want to accel
     */
    public void beginAcceleration(double targetVel) {
        beginAcceleration(targetVel, targetVel);
    }

    /**
     * Begin accelerating to a speed, specifying both the speed you actually want to send to the motors and the measured speed you want to actually reach
     * @param targetVel The velocity to command to the motors
     * @param checkValue The value to check the velocity against (in the case of a stop where we commandn a non-zero vel but want the actual vel to be 0)
     */
    public void beginAcceleration(double targetVel, double checkValue) {
        targetVel_Revs = targetVel;
        targetVelCheckValue_Revs = checkValue;
        accelerating = true;
    }

    /**
     * Zero's the encoders
     */
    public void zeroEncoders() {
        encoder_left.setPosition(0);
        encoder_right.setPosition(0);
    }

    /**
     * Tells whether we are at our target velocity (compares ACTUAL vel to the CHECK VALUE, which could be different (e.g. in the case of a stop))
     * @return Whether we're within tolerance of our target value
     */
    public boolean atTargetVel() {
        return valueIsWithinTolerance(encoder_left.getVelocity(), targetVelCheckValue_Revs, Constants.Elevator.atVelocityToleranceRevs);
    }

    public boolean valueIsWithinTolerance(double val, double target, double tolerance) {
        var absTarget = Math.abs(target);
        var absVal = Math.abs(val);
        return Math.abs(absTarget - absVal) < tolerance;
    }

    @Override
    public void periodic() {
        //Safeguard against going too high
        if (currentPosition_Inches() > Constants.Elevator.heightMax_Inches && currentVelocity_InchesPerSec() > 0.1) {
            stop();
            moveInProgress = false; 
        }

        if (moveInProgress) {
            if (atTargetPosition()) {
                stop();
                moveInProgress = false;
            }
        }

        if (accelerating) {
            if (!atTargetVel()) {
                motor_left.set(rateLimiterLeft.calculate(targetVel_Revs));
            }
            else {
                accelerating = false;
            }
        }

        SmartDashboard.putNumber("Elevator Height (in.)", currentPosition_Inches());
    }
}