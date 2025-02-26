package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Climb extends SubsystemBase {

    private final SparkMax climbMotor;

    public Climb(){
        climbMotor = new SparkMax(ClimbConstants.climbMotorID, MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .inverted(ClimbConstants.climbMotorInverted);
    }

    public void climbUp() {
        climbMotor.set(ClimbConstants.speed);
    }

    //method for getting climb to retract - not sure if it would function as desired 
    public void climbDown() {
        // climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        climbMotor.set(ClimbConstants.speed);
    }

    public void turnOff() {
        climbMotor.set(0);
    }

    @Override
    public void periodic() {
    }


}
