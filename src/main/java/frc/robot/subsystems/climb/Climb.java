package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


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

    public void disable() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disable'");
    }

    public void enable() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enable'");
    }


}
