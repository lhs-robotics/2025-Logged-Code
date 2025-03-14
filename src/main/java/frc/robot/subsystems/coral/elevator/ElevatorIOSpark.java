package frc.robot.subsystems.coral.elevator;

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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax motor1;
  private final SparkMax motor2;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController elevatorController;

  // For checking if Max has any faults in the span of .5 secs, even if they go
  // away
  // (To check for flashing faults / kind of connected motor)
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  // Records current setpoint to be able to check if elevator is at it (DO NOT
  // MODIFY)
  private double heighSetpoint = 0.0;

  ElevatorFeedforward feedForward;

  public ElevatorIOSpark() {
    // You know what this one does
    motor1 = new SparkMax(ElevatorConstants.motor1ID, MotorType.kBrushless);
    motor2 = new SparkMax(ElevatorConstants.motor2ID, MotorType.kBrushless);

    // Spark Max built in relative encoder - position is 0 at startup

    // Sparkle Maximum built in PID Controller
    elevatorController = motor1.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Basic motor setup (current limit is what motor will max out at, even if code
    // tells it more)
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.elevatorMotorSmartCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(ElevatorConstants.motor1Inveted);

    // Converts from motor rotation to height gained in inches
    // TODO: CALCULATE THIS CONVERSION FACTOR

    motorConfig.encoder.positionConversionFactor(ElevatorConstants.positionConversionFactor);
    motorConfig.encoder.velocityConversionFactor(ElevatorConstants.positionConversionFactor);

    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Position Control PID
        .p(ElevatorConstants.positionP, ClosedLoopSlot.kSlot0)
        .i(ElevatorConstants.positionI, ClosedLoopSlot.kSlot0)
        .d(ElevatorConstants.positionD, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);
    motorConfig.closedLoop
        .p(ElevatorConstants.velocityP, ClosedLoopSlot.kSlot1)
        .i(ElevatorConstants.velocityI, ClosedLoopSlot.kSlot1)
        .d(ElevatorConstants.velocityD, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
    // // These are the speeds max motion will attempt to achieve (not maximum it
    // will
    // // go to, what it will always go to )
    .maxVelocity(ElevatorConstants.maxVelocity)
    .maxAcceleration(ElevatorConstants.maxAcceleration)
    .allowedClosedLoopError(ElevatorConstants.allowedError);

    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig motorBConfig = new SparkMaxConfig();
    motorBConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.elevatorMotorSmartCurrentLimit)
        .voltageCompensation(12.0)
        .inverted(ElevatorConstants.elevatorMotor2Inverted)
        .follow(motor1);
    // Modify motor config for motor 2
    motor2.configure(motorBConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = motor1.getEncoder();

    // Elevator Boot Location is 0
    encoder.setPosition(0);

    // Feedforward compensates for gravity - even if it is not moving the motor
    // needs some voltage to keep the elevator from sliding down
    feedForward = new ElevatorFeedforward(
        ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  }

  /**
   * Passes physical data back to Elevator.java for logging and other use
   */
  public void updateInputs(ElevatorIOInputs inputs) {

    sparkStickyFault = false;
    ifOk(motor1, encoder::getPosition, (value) -> inputs.height = value);
    ifOk(motor1, encoder::getVelocity, (value) -> inputs.velocityRPM = value);
    inputs.atTarget = checkAtTarget();
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  /**
   * Returns if elevator is within ElevatorConstants.allowedError of target
   * setpoint for position (NOT VELOCITY)
   */
  public boolean checkAtTarget() {
    double currPos = encoder.getPosition();
    return Math.abs(heighSetpoint - currPos) < ElevatorConstants.allowedError;
  }

  @Override
  /** Sets elevator height from elevator bootup 0 (not robot base) */
  public void setElevatorHeightInches(double heightInches) {
    // Feedforward compensates for gravity - even if it is not moving the motor
    // needs some voltage to keep the elevator from sliding down
    // In this case it also is used to determine how many additional volts it likely
    // needs to accelerate to movement (acceleration takes much more energy than
    // cruise)
    double feedforward = feedForward.calculate(ElevatorConstants.maxVelocity);

    heighSetpoint = heightInches;

    elevatorController.setReference(
        heightInches, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0);
  }

  /**
   * Sets elevator run at a specificied voltage - FOR SYSID USE ONLY
   * 
   * @param output Voltage to run at
   */
  public void runCharacterization(double output) {
    motor1.setVoltage(output);
  }

  /**
   * Uses Max Motion PID to run elevator at specified velocity
   * Speeds up in accordance to the max acceleration variable
   * 
   * @param velocity Desired velocity - currently in RPM (2/14/25) however
   *                 eventually should be in inches/minute
   */
  @Override
  public void setVelocity(double velocity) {
    // elevatorController.setReference(
    // velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    // TODO: ADD PROTECTION FROM HITTING TOP OR BOTTOM
  }

  /**
   * Overrides the max velocity and acceleration set in elevator constants
   * 
   * @param velocity     Desired new max velocity - currently in Revolutions per
   *                     minute (2/14/25) however eventually should be in
   *                     inches/minute
   * @param acceleration Desired new max velocity - currently in RPM per second
   *                     (2/14/25) however eventually should be in inches/second
   */
  @Override
  public void setMaxVelocityAcceleration(double velocity, double acceleration) {
    // SparkMaxConfig newConfig = new SparkMaxConfig();

    // newConfig.closedLoop.maxMotion.maxVelocity(velocity).maxAcceleration(acceleration);

    // motor1.configure(newConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kNoPersistParameters);
  }

  /**
   * Overrides the position movement PID set in elevator constants
   */
  @Override
  public void setPositionPID(double kP, double kI, double kD) {
    // SparkMaxConfig newConfig = new SparkMaxConfig();

    // newConfig.closedLoop
    // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // // Position Control PID
    // .p(ElevatorConstants.positionP)
    // .i(ElevatorConstants.positionI)
    // .d(ElevatorConstants.positionD)
    // .outputRange(-1, 1);

    // motor1.configure(newConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kNoPersistParameters);
  }

  @Override
  public void manualElevatorStart(boolean up) {
    if (up) {
      motor1.set(0.40);
    } else {
      motor1.set(-0.40);
    }
  }

  @Override
  public void manualElevatorStop() {
    motor1.set(0);
    elevatorController.setReference(encoder.getPosition(), ControlType.kPosition);
  }

}
