package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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
        motorRight.set(-Constants.CoralManipulator.expelCoralSpeed * 0.5);
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
     * True if the left motor is moving within a tolerance of its ingest speed
     * @return
     */
    public boolean isIngestingCoral() {
        return motorLeft.get() > (Constants.CoralManipulator.ingestCoralSpeed - Constants.CoralManipulator.speedCheckMargin) && motorLeft.get() < (Constants.CoralManipulator.ingestCoralSpeed - Constants.CoralManipulator.speedCheckMargin);
    }

    /**
     * True if the left motor is moving within a tolerance of its expel speed
     * @return
     */
    public boolean isExpelingCoral() {
        return motorLeft.get() > (Constants.CoralManipulator.expelCoralSpeed - Constants.CoralManipulator.speedCheckMargin) && motorLeft.get() < (Constants.CoralManipulator.expelCoralSpeed - Constants.CoralManipulator.speedCheckMargin); 
    }
}
