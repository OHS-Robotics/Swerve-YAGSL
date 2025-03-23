package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

public class CoralManipulatorSubsystem extends SubsystemBase{
    private SparkMax motorLeft = new SparkMax(Constants.CANIDs.CoralManipulatorMotorLeft, MotorType.kBrushless);
    private SparkMax motorRight = new SparkMax(Constants.CANIDs.CoralManipulatorMotorRight, MotorType.kBrushless);
    public final LaserCan laser = new LaserCan(Constants.CANIDs.CoralManipulatorLaser);

    public CoralManipulatorSubsystem() {
        try {
            laser.setRangingMode(RangingMode.SHORT);
        }
        catch (Exception ex) {
            System.out.println("Laser Configuration Failed");
        }
    }
    
    public void ingestCoral() {
        motorLeft.set(Constants.CoralManipulator.ingestCoralSpeed);
        motorRight.set(-Constants.CoralManipulator.ingestCoralSpeed);
    }

    public void expelCoral() {
        motorLeft.set(Constants.CoralManipulator.expelCoralSpeed);
        motorRight.set(-Constants.CoralManipulator.expelCoralSpeed);
    }

    public void expelCoralTwist() {
        motorLeft.set(Constants.CoralManipulator.expelCoralSpeed);
        motorRight.set(-Constants.CoralManipulator.expelCoralSpeed * Constants.CoralManipulator.expalCoralTwistRightMotorScale);
    }

    public void stopMoving() {
        motorLeft.set(0);
        motorRight.set(0);
    }

    public boolean CoralLoaded() {
        if (RobotBase.isReal()) {
            var dist = laser.getMeasurement();
            if (dist == null) dist = new Measurement(0, 0, 0, false, 0, null);
            return dist.distance_mm < Constants.CoralManipulator.coralSenseDistance_mm;
        }
        return true;
    }

    /**
     * True if the motors are moving within a tolerance of their ingest speed
     * @return
     */
    public boolean isIngestingCoral() {
        return 
            valueIsWithinTolerance(motorLeft.getEncoder().getVelocity(), Constants.CoralManipulator.ingestCoralSpeed, Constants.CoralManipulator.speedCheckMargin) &&
            valueIsWithinTolerance(-motorRight.getEncoder().getVelocity(), Constants.CoralManipulator.ingestCoralSpeed, Constants.CoralManipulator.speedCheckMargin);
    }

    /**
     * True if the motors are moving within a tolerance of their expel speed
     * @return
     */
    public boolean isExpelingCoral() {
        return 
            valueIsWithinTolerance(motorLeft.getEncoder().getVelocity(), Constants.CoralManipulator.expelCoralSpeed, Constants.CoralManipulator.speedCheckMargin) &&
            valueIsWithinTolerance(-motorRight.getEncoder().getVelocity(), Constants.CoralManipulator.expelCoralSpeed, Constants.CoralManipulator.speedCheckMargin);
    }

    /**
     * True if the motors are moving within a tolerance of their expel twist speed
     * @return
     */
    public boolean isExpelingCoralTwist() {
        return 
            valueIsWithinTolerance(motorLeft.getEncoder().getVelocity(), Constants.CoralManipulator.expelCoralSpeed, Constants.CoralManipulator.speedCheckMargin) &&
            valueIsWithinTolerance(-motorRight.getEncoder().getVelocity(), Constants.CoralManipulator.expelCoralSpeed * Constants.CoralManipulator.expalCoralTwistRightMotorScale, Constants.CoralManipulator.speedCheckMargin);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Coral Manipulator", new CoralManipulatorData(this));
    }

    public boolean valueIsWithinTolerance(double val, double target, double tolerance) {
        var absTarget = Math.abs(target);
        var absVal = Math.abs(val);
        return Math.abs(absTarget - absVal) < tolerance;
    }

    class CoralManipulatorData implements Sendable{
        CoralManipulatorSubsystem coralManip;

        CoralManipulatorData(CoralManipulatorSubsystem coralManip) {
            this.coralManip = coralManip;
        }

        boolean getIsIngestingCoral() {
            return coralManip.isIngestingCoral();
        }

        boolean getIsExpelingCoral() {
            return coralManip.isExpelingCoral();
        }

        boolean getIsExpellingCoralTwist() {
            return coralManip.isExpelingCoralTwist();
        }

        double getLaserDistance_mm() {
            return coralManip.laser.getMeasurement().distance_mm;
        }


        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Coral Manip");
            builder.addBooleanProperty("Ingesting", this::getIsIngestingCoral, null);
            builder.addBooleanProperty("Expelling", this::getIsExpelingCoral, null);
            builder.addBooleanProperty("Expelling Twist", this::getIsExpellingCoralTwist, null);
            builder.addDoubleProperty("Laser Dist (mm)", this::getLaserDistance_mm, null);
        }
    }
}
