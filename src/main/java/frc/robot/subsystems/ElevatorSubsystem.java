package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motor_left = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax motor_right = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
    
    private final SparkClosedLoopController clc_left = motor_left.getClosedLoopController();
    private final SparkClosedLoopController clc_right = motor_right.getClosedLoopController();

    // left is the forward one, right is the actual one
    private final RelativeEncoder encoder_left = motor_left.getEncoder();
    private final RelativeEncoder encoder_right = motor_right.getEncoder();

    private double expectedPosition = 0.0;

    // private final PIDController pid = new PIDController(0.1, 0.0, 0.0);

    public ElevatorSubsystem() {
        
        final SparkMaxConfig baseconf_left = new SparkMaxConfig();
        final SparkMaxConfig baseconf_right = new SparkMaxConfig();
        baseconf_right.follow(motor_right, true);
        baseconf_left.smartCurrentLimit(30);
        baseconf_right.smartCurrentLimit(30);
        baseconf_left.idleMode(IdleMode.kBrake);
        baseconf_right.idleMode(IdleMode.kBrake);
        baseconf_right.encoder.inverted(true);

        baseconf_left.closedLoop.pid(0.1, 0.0 , 0.0).outputRange(0.0, 100.0);
        baseconf_right.closedLoop.pid(0.1, 0.0, 0.0).outputRange(0.0, 100.0);
        // baseconf_left.closedLoop.maxMotion.maxVelocity(0);
        // remember to add limits later

        motor_left.configure(baseconf_left, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motor_right.configure(baseconf_right, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // clc_left.setReference(0.1, ControlType.kPosition);
    }
    
    public void setPosition(double position) { // position is in motor rotations
        // todo: calculate elevator change from motor rotation
        // double output = pid.calculate((encoder_left.getPosition() + encoder_right.getPosition())/2.0, position);
        
        // inPosition = Math.abs((encoder_left.getPosition() + encoder_right.getPosition())/2.0 - output) < 0.1;

        // motor_left.set((encoder_left.getPosition() - output) * 0.1);
        // motor_right.set((encoder_right.getPosition() - output) * -0.1);
        expectedPosition = position;
        clc_left.setReference(position, ControlType.kPosition); // right motor is a follower
    }

    public void resetEncoders() {
        encoder_left.setPosition(0);
        encoder_right.setPosition(0);
    }

    public boolean positioned() {  
        return Math.abs((encoder_left.getPosition() + encoder_right.getPosition())/2.0 - expectedPosition) < 0.1;
    }

}
