package frc.robot.subsystems;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motor_left = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);

    // IMPORTANT NOTE: Right motor is configured as a follower of the left motor.  We will only command moves to the left motor

    private double k_atVelocityToleranceRevs = 0.01;
    private double k_maxVel = 0.1;
    private double k_maxAccel = 0.1;
    private double targetVel = 0;
    
    private final SparkClosedLoopController clc_left = motor_left.getClosedLoopController();
    // private final SparkClosedLoopController clc_right = motor_right.getClosedLoopController();

    SlewRateLimiter rateLimiterLeft = new SlewRateLimiter(k_maxAccel);
    SlewRateLimiter rateLimiterRight = new SlewRateLimiter(k_maxAccel);

    // left is the forward one, right is the actual one
    private final RelativeEncoder encoder_left = motor_left.getEncoder();
    private final RelativeEncoder encoder_right = motor_right.getEncoder();

    public double targetPositionRevs = 0;
    public boolean moveInProgress = false;

    // private final PIDController pid = new PIDController(0.1, 0.0, 0.0);

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
        
        baseconf_right.encoder.inverted(true);

        // baseconf_left.closedLoop.pid(0.01, 0.01 , 0.0).outputRange(-k_maxVel, k_maxVel);
        // baseconf_right.closedLoop.pid(0.1, 0.0, 0.0).outputRange(-1, 1);
        baseconf_left.closedLoop.maxMotion.maxVelocity(k_maxVel);
        // remember to add limits later

        motor_left.configure(baseconf_left, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // motor_right.configure(baseconf_right, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double currentPosition() {
        return encoder_left.getPosition();
    }
    
    public void update() {
        SmartDashboard.putNumber("Elevator Left", encoder_left.getPosition());
        SmartDashboard.putNumber("Elevator Right", encoder_right.getPosition());
    }

    // todo: test these
    /**
     * Begins a move to an absolute position.  Move Absolute Complete must be checked in order to know when it's complete
     * @param commandPosition Desired position (inches)
     */
    public void moveAbsoluteBegin(double positionInches, double speed) { // position is in inches
        // double output = pid.calculate((encoder_left.getPosition() + encoder_right.getPosition())/2.0, position);
        
        // inPosition = Math.abs((encoder_left.getPosition() + encoder_right.getPosition())/2.0 - output) < 0.1;

        // motor_left.set((encoder_left.getPosition() - output) * 0.1);
        // motor_right.set((encoder_right.getPosition() - output) * -0.1);
        // expectedPositionInches = commandPosition;

        // clc_left.setReference(commandPosition * k_revsPerInch, ControlType.kPosition); // right motor is a follower

        var positionRevs = positionInches * Constants.Elevator.revsPerInch;
        if (positionRevs > currentPosition()) {
            jogUp(speed);
        }
        else {
            jogDown(speed);
        }

        targetPositionRevs = positionRevs;
    }

    public boolean moveComplete() {
        return valueIsWithinTolerance(currentPosition(), targetPositionRevs, 0.5);
    }

    /*
     * Move relative to the elevator's current position
     */
    public void moveRelativeBegin(double distanceInches, double speed) { // commandDistance is in Inches
        // double output = pid.calculate((encoder_left.getPosition() + encoder_right.getPosition())/2.0, position);
        
        // inPosition = Math.abs((encoder_left.getPosition() + encoder_right.getPosition())/2.0 - output) < 0.1;

        // motor_left.set((encoder_left.getPosition() - output) * 0.1);
        // motor_right.set((encoder_right.getPosition() - output) * -0.1);
        var targetPositionInches = (distanceInches * Constants.Elevator.revsPerInch + currentPosition()) / Constants.Elevator.revsPerInch;
        moveAbsoluteBegin(targetPositionInches, speed);
    }


    /**
     * Jog the elevator up
     * @param speed The speed at which you want to move, in inches/sec
     */
    public void jogUp(double speed) {
        if (speed < 0) {
            System.out.println("Cannot jog with a negative vel.  Provide only positive values");
            return;
        }
        
        // clc_left.setReference(speed * k_revsPerInch, ControlType.kVelocity);
        targetVel = speed * Constants.Elevator.revsPerInch;
        // motor_left.set(rateLimiterLeft.calculate(speed));
        motor_left.set(targetVel);
    }

    /**
     * Jog the elevator up
     * @param speed The speed at which you want to move, in inches/sec
     */
    public void jogDown(double speed) {
        if (speed < 0) {
            System.out.println("Cannot jog with a negative vel.  Provide only positive values");
            return;
        }

        // clc_left.setReference(-1 * speed * k_revsPerInch, ControlType.kVelocity);
        targetVel = -speed * Constants.Elevator.revsPerInch;
        // motor_left.set(rateLimiterLeft.calculate(-speed));
        motor_left.set(targetVel);
    }

    /**
     * Stops elevator motion
     */
    public void stop() {
        targetVel = 0;
        motor_left.set(Constants.Elevator.stopVelInchesPerSec * Constants.Elevator.revsPerInch);
    }


    public void zeroEncoders() {
        encoder_left.setPosition(0);
        encoder_right.setPosition(0);
    }

    public boolean isAtSpeed() {
        return valueIsWithinTolerance(encoder_left.getVelocity(), targetVel, k_atVelocityToleranceRevs);
    }

    public boolean valueIsWithinTolerance(double val, double target, double toleranceRevs) {
        var absTarget = Math.abs(target);
        var absVal = Math.abs(val);
        return Math.abs(absTarget - absVal) < toleranceRevs;
    }

    @Override
    public void periodic() {
        if (moveInProgress) {
            if (moveComplete()) {
                stop();
                moveInProgress = false;
            }
        }
    }
}