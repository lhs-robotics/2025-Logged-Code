package frc.robot.subsystems.coral.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.elevator.ElevatorConstants;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

public class ArmIOSpark implements ArmIO {
    private final SparkMax gearboxSpark;
    private final RelativeEncoder gearboxEncoder;
    private final SparkClosedLoopController gearboxPID;
    private double angleSetpoint = 0.0;

    private final SparkMax endAffectorMotor;

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            ArmConstants.kS,
            ArmConstants.kG,
            ArmConstants.kA);

    private final ProfiledPIDController m_controller = new ProfiledPIDController(ArmConstants.positionP,
            0,
            ArmConstants.positionD,
            new Constraints(ArmConstants.maxVelocity,
                    ArmConstants.maxAcceleration));

    public ArmIOSpark() {
        gearboxSpark = new SparkMax(ArmConstants.gearboxSparkID, MotorType.kBrushless);
        endAffectorMotor = new SparkMax(ArmConstants.endAffectorSparkID, MotorType.kBrushless);

        gearboxEncoder = gearboxSpark.getEncoder();
        gearboxPID = gearboxSpark.getClosedLoopController();

        SparkMaxConfig gearMotorConfig = new SparkMaxConfig();
        gearMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ArmConstants.gearBoxCurrentLimit)
                .voltageCompensation(12.0)
                .inverted(ArmConstants.gearBoxInveted);

        gearMotorConfig.encoder.positionConversionFactor(Units.rotationsToDegrees((72.0/42.0)/12.0));

        gearMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Position Control PID
                .p(ArmConstants.positionP)
                .i(0)
                .d(ArmConstants.positionD)
                .outputRange(-1, 1);
        gearMotorConfig.closedLoop
                .p(ArmConstants.velocityP, ClosedLoopSlot.kSlot1)
                .i(ArmConstants.velocityI, ClosedLoopSlot.kSlot1)
                .d(ArmConstants.velocityD, ClosedLoopSlot.kSlot1);
        gearMotorConfig.closedLoop.maxMotion.maxVelocity(400).maxAcceleration(350).allowedClosedLoopError(0.5);
        gearboxSpark.configure(gearMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        gearboxEncoder.setPosition(90);

        SparkMaxConfig endAffectorConfig = new SparkMaxConfig();
        endAffectorConfig.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(ArmConstants.endAffectorCurrentLimit)
                .voltageCompensation(12.0)
                .inverted(ArmConstants.endAffectorInverted);
        endAffectorMotor.configure(endAffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean checkAtTarget() {
        double currPos = gearboxEncoder.getPosition();
        return Math.abs(angleSetpoint - currPos) < ElevatorConstants.allowedError;
    }

    @Override
    public void runCharacterization(double output) {
        gearboxSpark.setVoltage(output);
    }

    private void reachGoal(double goal) {
        double voltsOut = MathUtil.clamp(
                m_controller.calculate(gearboxEncoder.getPosition(), goal) +
                    m_feedforward.calculate(Units.degreesToRadians(gearboxEncoder.getPosition()),
                                m_controller.getSetpoint().velocity),
                -12,
                12); // 7 is the max voltage to send out.
        gearboxSpark.setVoltage(voltsOut);
    }

    @Override
    public void setArmAngleDegrees(double angleDegrees) {
        // angleSetpoint = angleDegrees;
        // gearboxPID.setReference(angleDegrees, ControlType.kMAXMotionPositionControl,
        // ClosedLoopSlot.kSlot0);

        Commands.run(() -> reachGoal(angleDegrees)).schedule();

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
        ifOk(gearboxSpark, gearboxEncoder::getPosition, (value) -> inputs.angle = value);
        ifOk(gearboxSpark, gearboxEncoder::getVelocity, (value) -> inputs.gearboxVelocityRPM = value);
        ifOk(endAffectorMotor, endAffectorMotor::get, (value) -> inputs.endAffectorMotorSpeed = value);
        inputs.atTarget = checkAtTarget();
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void disableEndAffectorBrake() {
        SparkMaxConfig endAffectorConfig = new SparkMaxConfig();
        endAffectorConfig.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(ArmConstants.endAffectorCurrentLimit)
                .voltageCompensation(12.0)
                .inverted(ArmConstants.endAffectorInverted);
        endAffectorMotor.configure(endAffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void enableEndAffectorBrake() {
        SparkMaxConfig endAffectorConfig = new SparkMaxConfig();
        endAffectorConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ArmConstants.endAffectorCurrentLimit)
                .voltageCompensation(12.0)
                .inverted(ArmConstants.endAffectorInverted);
        endAffectorMotor.configure(endAffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setEndAffectorSpeed(double speed) {
        endAffectorMotor.set(speed);
    }

    @Override
    public void manualRunArm(boolean up) {
        if (up) {
            gearboxSpark.set(0.2);
        } else {
            gearboxSpark.set(-0.2);
        }
    }

    @Override
    public void manualArmStop() {
        gearboxSpark.set(0);
        gearboxPID.setReference(gearboxEncoder.getPosition(), ControlType.kPosition);
    }

}
