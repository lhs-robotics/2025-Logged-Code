package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

public class ClimbIOSpark implements ClimbIO {
    private SparkFlex climbMotor;
    private SparkClosedLoopController PIDController;
    private RelativeEncoder encoder;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    private double targetDegrees = 0;

    public ClimbIOSpark() {
        climbMotor = new SparkFlex(ClimbConstants.climbMotorID, MotorType.kBrushless);
        SparkFlexConfig motorConfig = new SparkFlexConfig();

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
        // Use MaxMotion PID which relies on restricting velocity rather than seeking to
        // get to a certain point
        // This means more power when actually climbing and less power when just moving
        // inside bumper
        targetDegrees = angle;
        PIDController.setReference(angle, ControlType.kPosition);
    }

    public boolean checkAtTarget() {
        double currPos = encoder.getPosition();
        return Math.abs(targetDegrees - currPos) < ClimbConstants.allowedError;
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(climbMotor, encoder::getPosition, (value) -> inputs.positionDegrees = value);
        ifOk(climbMotor, encoder::getVelocity, (value) -> inputs.velocityRPM = value);
        inputs.atTarget = checkAtTarget();
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }

}
