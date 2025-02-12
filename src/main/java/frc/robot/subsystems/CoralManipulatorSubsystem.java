package frc.robot.subsystems;

import javax.naming.ldap.SortResponseControl;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.external.LidarSubsystem;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

public class CoralManipulatorSubsystem extends SubsystemBase{
    private SparkMax motorLeft = new SparkMax(2, MotorType.kBrushless);
    private SparkMax motorRight = new SparkMax(3, MotorType.kBrushless);
    public final LaserCan laser = new LaserCan(38);
    
    private final double ingestCoralSpeed = 0.2;
    private final double expellCoralSpeed = 0.3;
    private final double speedCheckMargin = 0.05;
    private final double senseCoralDist = 30;

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

    public void expellCoral() {
        motorLeft.set(expellCoralSpeed);
        motorRight.set(-expellCoralSpeed);
    }

    public void stopMoving() {
        motorLeft.set(0);
        motorRight.set(0);
    }

    public boolean CoralLoaded() {
        var dist = laser.getMeasurement();
        System.out.println(dist.distance_mm);
        return dist.distance_mm < senseCoralDist;
    }

    /**
     * True if the left motor is moving within a tolerance of its ingeest speed
     * @return
     */
    public boolean isIngestingCoral() {
        return motorLeft.get() > (ingestCoralSpeed - speedCheckMargin) && motorLeft.get() < (ingestCoralSpeed - speedCheckMargin);
    }

    /**
     * True if the left motor is moving within a tolerance of its expell speed
     * @return
     */
    public boolean isExpellingCoral() {
        return motorLeft.get() > (expellCoralSpeed - speedCheckMargin) && motorLeft.get() < (expellCoralSpeed - speedCheckMargin); 
    }
}
