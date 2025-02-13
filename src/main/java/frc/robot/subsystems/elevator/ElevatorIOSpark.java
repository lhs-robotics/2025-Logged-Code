package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.*;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController elevatorController;
    
    private final Debouncer connectedDebounce = new Debouncer(0.5);

    ElevatorFeedforward feedForward;

    public ElevatorIOSpark() {
        motor1 = new SparkMax(ElevatorConstants.motor1ID, MotorType.kBrushless);
        motor2 = new SparkMax(ElevatorConstants.motor2ID, MotorType.kBrushless);

        encoder = motor1.getEncoder();
        elevatorController = motor1.getClosedLoopController();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.elevatorMotorSmartCurrentLimit)
                .voltageCompensation(12.0)
                .inverted(ElevatorConstants.motor1Inveted);

        motorConfig.encoder.positionConversionFactor(ElevatorConstants.positionConversionFactor);

        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Position Control PID
                .p(ElevatorConstants.positionP)
                .i(ElevatorConstants.positionI)
                .d(ElevatorConstants.positionD)
                .outputRange(-1, 1);

        motorConfig.closedLoop.maxMotion
                .maxVelocity(ElevatorConstants.maxVelocity)
                .maxAcceleration(ElevatorConstants.maxAcceleration)
                .allowedClosedLoopError(ElevatorConstants.allowedError);

        motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Modify motor config for motor 2
        motorConfig.inverted(ElevatorConstants.elevatorMotor2Inverted).follow(motor1);
        motor2.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Elevator Boot Location is 0
        encoder.setPosition(0);

        feedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV,
                ElevatorConstants.kA);
    }

    public void updateInputs(ElevatorIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(motor1, encoder::getPosition, (value) -> inputs.height = value);
        ifOk(motor1, encoder::getVelocity, (value) -> inputs.velocityRPM = value);
        inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    }

    /**
     * Sets elevator height from elevator bootup 0 (not robot base)
     */
    public void setElevatorHeightInches(double heightInches) {
        // Feedforward -> elevator go vroom vroom (accuratley)
        double feedforward = feedForward.calculate(ElevatorConstants.maxVelocity);

        elevatorController.setReference(heightInches, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                feedforward);
    }

    public void runCharacterization(double output) {
        motor1.setVoltage(output);
    }

    public void setVelocity(double velocity) {
        elevatorController.setReference(velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    }

}
