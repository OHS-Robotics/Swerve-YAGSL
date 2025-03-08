package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

public class CoralManipulatorSubsystem extends SubsystemBase{
    private SparkMax motorLeft = new SparkMax(2, MotorType.kBrushless);
    private SparkMax motorRight = new SparkMax(3, MotorType.kBrushless);
    public final LaserCan laser = new LaserCan(38);
    
    private final double ingestCoralSpeed = -0.15;
    private final double expelCoralSpeed = -0.75;
    private final double speedCheckMargin = 0.05;

    public CoralManipulatorSubsystem() {
        try {
            laser.setRangingMode(RangingMode.SHORT);
        }
        catch (Exception ex) {
            System.out.println("Laser Configuration Failed");
        }
    }
    
    public void ingestCoral() {
        motorLeft.set(ingestCoralSpeed);
        motorRight.set(-ingestCoralSpeed);
    }

    public void expelCoral() {
        motorLeft.set(expelCoralSpeed);
        motorRight.set(-expelCoralSpeed);
    }

    public void expelCoralTwist() {
        motorLeft.set(expelCoralSpeed);
        motorRight.set(-expelCoralSpeed * 0.5);
    }

    public void stopMoving() {
        motorLeft.set(0);
        motorRight.set(0);
    }

    public boolean CoralLoaded() {
        var dist = laser.getMeasurement();
        if (dist == null) dist = new Measurement(0, 0, 0, false, 0, null);
        return dist.distance_mm < Constants.CoralManipulator.coralSenseDistance_mm;
    }

    /**
     * True if the left motor is moving within a tolerance of its ingest speed
     * @return
     */
    public boolean isIngestingCoral() {
        return motorLeft.get() > (ingestCoralSpeed - speedCheckMargin) && motorLeft.get() < (ingestCoralSpeed - speedCheckMargin);
    }

    /**
     * True if the left motor is moving within a tolerance of its expel speed
     * @return
     */
    public boolean isExpelingCoral() {
        return motorLeft.get() > (expelCoralSpeed - speedCheckMargin) && motorLeft.get() < (expelCoralSpeed - speedCheckMargin); 
    }
}
