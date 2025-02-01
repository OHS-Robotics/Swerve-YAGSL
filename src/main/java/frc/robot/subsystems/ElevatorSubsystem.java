package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motor_left = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);

    // left is the forward one, right is the actual one
    private final RelativeEncoder encoder_left = motor_left.getEncoder();
    private final RelativeEncoder encoder_right = motor_right.getEncoder();

    private final PIDController pid = new PIDController(0.1, 0.0, 0.0);

    private boolean inPosition = false;

    public ElevatorSubsystem() {}
    
    public void setPosition(double position) { // position is in motor rotations
        // todo: calculate elevator change from motor rotation
        double output = pid.calculate((encoder_left.getPosition() + encoder_right.getPosition())/2.0, position);
        
        inPosition = Math.abs((encoder_left.getPosition() + encoder_right.getPosition())/2.0 - output) < 0.1;

        motor_left.set((encoder_left.getPosition() - output) * 0.1);
        motor_right.set((encoder_right.getPosition() - output) * -0.1);
    }

    public void resetEncoders() {
        encoder_left.setPosition(0);
        encoder_right.setPosition(0);
    }

    public void positioned() {

    }

}
