package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.external.LidarSubsystem;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralLoaderSubsystem extends SubsystemBase {

    private boolean coralLoaderRunning = false;
    private final SparkMax m_coralLoader = new SparkMax(6, SparkLowLevel.MotorType.kBrushed);
    public final LidarSubsystem lidar = new LidarSubsystem(Port.kOnboard);

    public CoralLoaderSubsystem() {}

    public Command toggleLoader() {
        return this.runOnce(() -> coralLoaderRunning = !coralLoaderRunning);
    }

    private void loaderFunction() {
        int dist = lidar.getDistance();

        if (coralLoaderRunning && dist < 65) {
        coralLoaderRunning = false;
        }

        if (coralLoaderRunning) {
            m_coralLoader.set(.5);

        } else {
            m_coralLoader.set(0.0);
        }
    }

    public Command runLoader() {
        return this.runOnce(this::loaderFunction);
    }

    
}
