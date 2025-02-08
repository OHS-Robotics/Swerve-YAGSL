package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.external.LidarSubsystem;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

public class CoralManipulatorSubsystem extends SubsystemBase{
    private SparkMax motor = new SparkMax(2, MotorType.kBrushless);
    public final LidarSubsystem lidar = new LidarSubsystem(Port.kOnboard);
    public final LaserCan laser = new LaserCan(37);
    
    private final double ingestCoralSpeed = 0.05;
    private final double expellCoralSpeed = 0.05;
    private final double speedCheckMargin = 0.01;
    private final double senseCoralDist = 65;

    public CoralManipulatorSubsystem() {
        lidar.startMeasuring();
        try {
            laser.setRangingMode(RangingMode.SHORT);
        }
        catch (Exception ex) {
            System.out.println("Laser Configuration Failed");
        }
    }
    
    public void ingestCoral() {
        motor.set(ingestCoralSpeed);
    }

    public void expellCoral() {
        motor.set(expellCoralSpeed);
    }

    public void stopMoving() {
        motor.set(0);
    }

    public boolean CoralLoaded() {
        // var dist = lidar.getDistance();
        var dist = laser.getMeasurement();
        System.out.println(dist.distance_mm);
        return dist.distance_mm < 50;
    }

    public boolean isIngestingCoral() {
        return motor.get() > (ingestCoralSpeed - speedCheckMargin) && motor.get() < (ingestCoralSpeed - speedCheckMargin); 
    }

    public boolean isExpellingCoral() {
        return motor.get() > (expellCoralSpeed - speedCheckMargin) && motor.get() < (expellCoralSpeed - speedCheckMargin); 
    }
}
