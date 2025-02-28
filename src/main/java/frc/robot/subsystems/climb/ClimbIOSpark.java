package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbIOSpark implements ClimbIO {
    private SparkMax climbMotor;
    private SparkClosedLoopController PIDController;
    private RelativeEncoder encoder;

    public ClimbIOSpark() {
        climbMotor = new SparkMax(ClimbConstants.climbMotorID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0)
                .inverted(ClimbConstants.climbMotorInverted);

        climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorConfig.encoder.positionConversionFactor(ClimbConstants.positionConversionFactor);
        motorConfig.encoder.velocityConversionFactor(ClimbConstants.velocityConversionFactor);

        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Position Control PID
                .p(ClimbConstants.positionP)
                .i(ClimbConstants.positionI)
                .d(ClimbConstants.positionD)
                .outputRange(-1, 1);

        motorConfig.closedLoop.maxMotion
                // These are the speeds max motion will attempt to achieve (not maximum it will
                // go to, what it will always go to )
                .maxVelocity(ClimbConstants.maxVelocity)
                .maxAcceleration(ClimbConstants.maxAcceleration)
                .allowedClosedLoopError(ClimbConstants.allowedError);

        PIDController = climbMotor.getClosedLoopController();
        encoder = climbMotor.getEncoder();
    }

    @Override
    public void setPositionDegrees(double angle) {
        // Use MaxMotion PID which relies on restricting velocity rather than seeking to get to a certain point
        // This means more power when actually climbing and less power when just moving inside bumper
        PIDController.setReference(angle, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        // TODO Auto-generated method stub
        ClimbIO.super.updateInputs(inputs);
    }

}
