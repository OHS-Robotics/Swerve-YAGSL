package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.external.LidarSubsystem;

public class CoralManipulatorSubsystem extends SubsystemBase{
    private SparkMax motor = new SparkMax(17, MotorType.kBrushless);
    public final LidarSubsystem lidar = new LidarSubsystem(Port.kOnboard);
    private final double ingestCoralSpeed = 0.1;
    private final double expellCoralSpeed = -0.1;
    private final double speedCheckMargin = 0.05;
    private final double senseCoralDist = 65;

    public CoralManipulatorSubsystem() {
        lidar.startMeasuring();
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
        var dist = lidar.getDistance();
        System.out.println(dist);
        return dist < senseCoralDist;
    }

    public boolean isIngestingCoral() {
        return motor.get() > (ingestCoralSpeed - speedCheckMargin) && motor.get() < (ingestCoralSpeed - speedCheckMargin); 
    }

    public boolean isExpellingCoral() {
        return motor.get() > (expellCoralSpeed - speedCheckMargin) && motor.get() < (expellCoralSpeed - speedCheckMargin); 
    }
}
