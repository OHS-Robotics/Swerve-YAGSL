package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.external.LidarSubsystem;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CoralManipulatorSubsystem extends SubsystemBase {
    private final SparkMax m_coralManipulator = new SparkMax(16, SparkLowLevel.MotorType.kBrushed);
    public final LidarSubsystem lidar = new LidarSubsystem(Port.kOnboard);
    private final double ingestCoralSpeed = 5;
    private final double expellCoralSpeed = -5;
    private final double speedCheckMargin = 0.1;

    public CoralManipulatorSubsystem() {}

    public void ingestCoral() {
        m_coralManipulator.set(ingestCoralSpeed);
    }

    public void expellCoral() {
        m_coralManipulator.set(expellCoralSpeed);
    }

    public boolean CoralLoaded() {
        return lidar.getDistance() < 65;
    }
    
    public boolean isIngestingCoral() {
        return m_coralManipulator.get() > (ingestCoralSpeed - speedCheckMargin) && m_coralManipulator.get() < (ingestCoralSpeed + speedCheckMargin);
    }

    public boolean isExpellingCoral() {
        return m_coralManipulator.get() > (expellCoralSpeed - speedCheckMargin) && m_coralManipulator.get() < (expellCoralSpeed + speedCheckMargin);
    }

    public void stopActions() {
        m_coralManipulator.stopMotor();
    }

    //push button
    //run motor until lidar detects coral
    //next button push
    //run the motors backwards until you DON"T detect coral
}