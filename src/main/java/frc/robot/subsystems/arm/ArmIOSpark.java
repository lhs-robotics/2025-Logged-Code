package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

import frc.robot.subsystems.elevator.ElevatorConstants;

public class ArmIOSpark implements ArmIO {
    private final SparkMax gearboxSpark;
    private final RelativeEncoder gearboxEncoder;
    private final SparkClosedLoopController gearboxPID;
    private double angleSetpoint = 0.0;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public ArmIOSpark() {
        gearboxSpark = new SparkMax(ArmConstants.gearboxSparkID, MotorType.kBrushless);

        gearboxEncoder = gearboxSpark.getEncoder();
        gearboxPID = gearboxSpark.getClosedLoopController();

        SparkMaxConfig gearMotorConfig = new SparkMaxConfig();
        gearMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ArmConstants.gearBoxCurrentLimit)
                .voltageCompensation(12.0)
                .inverted(ArmConstants.gearBoxInveted);

        gearMotorConfig.encoder.positionConversionFactor(ArmConstants.gearboxRatio);

        gearMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Position Control PID
                .p(ArmConstants.positionP)
                .i(0)
                .d(ArmConstants.positionD)
                .outputRange(-1, 1);
                
        gearboxSpark.configure(gearMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        gearboxEncoder.setPosition(0);
    }

    public boolean checkAtTarget() {
        double currPos = gearboxEncoder.getPosition();
        return Math.abs(angleSetpoint - currPos) < ElevatorConstants.allowedError;
    }

    @Override
    public void runCharacterization(double output) {
        gearboxSpark.setVoltage(output);
    }

    @Override
    public void setArmAngleDegrees(double angleDegrees) {
        angleSetpoint = angleDegrees;
        gearboxPID.setReference(angleDegrees, ControlType.kPosition);
    }

    @Override
    public void setMaxVelocityAcceleration(double velocity, double acceleration) {
    }

    @Override
    public void setPositionPID(double kP, double kI, double kD) {
        // TODO Auto-generated method stub
    }

    @Override
    public void setVelocity(double velocity) {
        // TODO Auto-generated method stub
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(gearboxSpark, gearboxEncoder::getPosition, (value) -> inputs.height = value);
        ifOk(gearboxSpark, gearboxEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
        inputs.atTarget = checkAtTarget();
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }

}
