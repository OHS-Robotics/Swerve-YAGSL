package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

    private SparkMax motorLift = new SparkMax(Constants.CANIDs.AlgaeManipulatorMotor, MotorType.kBrushless);
    private SlewRateLimiter rateLimiterLeft = new SlewRateLimiter(Constants.AlgaeManipulator.slewRate_DegPerSecPerSec * Constants.AlgaeManipulator.revsPerDegree);
    private final RelativeEncoder encoderLift = motorLift.getEncoder();

    DigitalInput limitHigh = new DigitalInput(Constants.AlgaeManipulator.limitIOSlotHigh);
    DigitalInput limitLow = new DigitalInput(Constants.AlgaeManipulator.limitIOSlotLow);

    public double targetPosition_Revs = 0;
    private double targetVel_Revs = 0;
    private double targetVelCheckValue_Revs = 0;
    public boolean moveInProgress = false;
    public boolean accelerating = false; //true when the robot is not at target vel, meaning the slew rate limiter is doing work
    private boolean joggingDown = false;
    private boolean joggingUp = false;

    public AlgaeManipulatorSubsystem() {
        final SparkMaxConfig baseConfLift = new SparkMaxConfig();
        zeroEncoders();
        baseConfLift.smartCurrentLimit(60);
        baseConfLift.idleMode(IdleMode.kBrake);
        motorLift.configure(baseConfLift, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double currentPosition_Revs() {
        return -encoderLift.getPosition();
    }

    public double currentPosition_Degrees() {
        return -encoderLift.getPosition() / Constants.AlgaeManipulator.revsPerDegree;
    }

    public double currentVelocity_RevsPerSec() {
        return -encoderLift.getVelocity();
    }

    public double currentVelocity_DegreesPerSec() {
        return -encoderLift.getVelocity() / Constants.AlgaeManipulator.revsPerDegree;
    }
    
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Algae Manipulator Lift Pos (deg)", currentPosition_Degrees());
    }

    public boolean atHighLimit() {
        return !limitHigh.get();
    }

    public boolean atLowLimit() {
        return !limitLow.get();
    }

    /**
     * Begins a move to an absolute position.  The elevator will stop by itself when it reaches its target position
     * @param commandPosition Desired position (inches)
     */
    public void moveAbsoluteBegin(double position_Deg, double speed_DegPerSecond) { // position is in inches
        targetPosition_Revs = position_Deg * Constants.AlgaeManipulator.revsPerDegree;
        targetVel_Revs = speed_DegPerSecond * Constants.AlgaeManipulator.revsPerDegree;
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
    public void moveRelativeBegin(double distance_Deg, double speed_DegPerSecond) { // commandDistance is in Inches
        var targetPositionDeg = (distance_Deg * Constants.AlgaeManipulator.revsPerDegree + currentPosition_Revs()) / Constants.Elevator.revsPerInch;
        moveAbsoluteBegin(targetPositionDeg, speed_DegPerSecond);
    }


    /**
     * Jog the elevator up
     * @param speed_DegPerSecond The speed at which you want to move, in inches/sec
     */
    public void jogUp(double speed_DegPerSecond) {
        if (speed_DegPerSecond < 0) {
            System.out.println("Cannot jog with a negative vel.  Provide only positive values");
            return;
        }

        //Up is negative for this elevator
        // beginAcceleration(-speed_DegPerSecond * Constants.AlgaeManipulator.revsPerDegree);
        motorLift.set(speed_DegPerSecond * Constants.AlgaeManipulator.revsPerDegree);
        joggingUp = true;
    }

    /**
     * Jog the elevator up
     * @param speed_DegPerSecond The speed at which you want to move, in inches/sec
     */
    public void jogDown(double speed_DegPerSecond) {
        if (speed_DegPerSecond < 0) {
            System.out.println("Cannot jog with a negative vel.  Provide only positive values");
            return;
        }

        //Down is positive for this elevator
        // beginAcceleration(speed_DegPerSecond * Constants.AlgaeManipulator.revsPerDegree);
        motorLift.set(-speed_DegPerSecond * Constants.AlgaeManipulator.revsPerDegree);
        joggingDown = true;
    }

    /**
     * Stops elevator motion
     */
    public void stop() {
        targetVel_Revs = Constants.AlgaeManipulator.revsPerDegree;
        // beginAcceleration(targetVel_Revs, 0);
        motorLift.set(Constants.AlgaeManipulator.speedHold_DegPerSec * Constants.AlgaeManipulator.revsPerDegree);
        joggingDown = false;
        joggingUp = false;
    }

    /**
     * Begin accelerating to a speed
     * @param targetVel_Revs The velocity to which we want to accel
     */
    public void beginAcceleration(double targetVel_Revs) {
        beginAcceleration(targetVel_Revs, targetVel_Revs);
    }

    /**
     * Begin accelerating to a speed, specifying both the speed you actually want to send to the motors and the measured speed you want to actually reach
     * @param targetVel_Revs The velocity to command to the motors
     * @param checkValue The value to check the velocity against (in the case of a stop where we commandn a non-zero vel but want the actual vel to be 0)
     */
    public void beginAcceleration(double targetVel_Revs, double checkValue) {
        this.targetVel_Revs = targetVel_Revs;
        targetVelCheckValue_Revs = checkValue;
        accelerating = true;
    }

    /**
     * Zero's the encoders
     */
    public void zeroEncoders() {
        encoderLift.setPosition(0);
    }

    /**
     * Tells whether we are at our target velocity (compares ACTUAL vel to the CHECK VALUE, which could be different (e.g. in the case of a stop))
     * @return Whether we're within tolerance of our target value
     */
    public boolean atTargetVel() {
        return valueIsWithinTolerance(encoderLift.getVelocity(), targetVelCheckValue_Revs, Constants.AlgaeManipulator.atVelocityToleranceRevs);
    }

    /**
     * Tells whether the algae manipulator's velocity is close to zero, within the threshold set in the constants file
     * @return Whether the algae manipulator is moving
     */
    public boolean isMoving() {
        return isMoving(Constants.AlgaeManipulator.atVelocityToleranceRevs);
    }

    /**
     * Tells whether the algae manipulator's velocity is close to zero, within a custom threshold
     * @return Whether the algae manipulator is moving
     */
    public boolean isMoving(double tolerance_RevsPerSecond) {
        return !valueIsWithinTolerance(currentVelocity_RevsPerSec(), 0, tolerance_RevsPerSecond);
    }

    public boolean isMovingUp() {
        return isMoving() && (currentVelocity_RevsPerSec() < 0);
    }

    public boolean isMovingDown() {
        return isMoving() && (currentVelocity_RevsPerSec() > 0);
    }

    public boolean valueIsWithinTolerance(double val, double target, double tolerance) {
        var absTarget = Math.abs(target);
        var absVal = Math.abs(val);
        return Math.abs(absTarget - absVal) < tolerance;
    }

    @Override
    public void periodic() {
        //Safeguard against going too high
        if (currentPosition_Degrees() > Constants.Elevator.heightSoftLimit_Inches && currentVelocity_DegreesPerSec() > 0.1) {
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
                motorLift.set(rateLimiterLeft.calculate(targetVel_Revs));
            }
            else {
                accelerating = false;
            }
        }

        if (isMovingUp() && atHighLimit() && !joggingDown) {
            stop();
        }

        if (isMovingDown() && atLowLimit() && !joggingUp) {
            stop();
        }

        SmartDashboard.putBoolean("Algae Limit High", atHighLimit());
        SmartDashboard.putBoolean("Algae Limit Low", atLowLimit());
        SmartDashboard.putBoolean("Algae Moving Up", isMovingUp());
        SmartDashboard.putBoolean("Algae Moving Down", isMovingDown());

        updateSmartDashboard();
    }
}