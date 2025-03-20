package frc.robot.subsystems.coral.elevator;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  final ElevatorIO elevatorIO;
  private final SysIdRoutine sysId;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  // Tunable Numbers - Change in Advantage Scope while code is in tunable mode
  // (set in overall robot
  // constant file)
  // This does not save the constants in ElevatorConstants.java - you must
  // manually do that
  private static final LoggedTunableNumber positionkP = new LoggedTunableNumber("Elevator/positionkP",
      ElevatorConstants.positionP);
  private static final LoggedTunableNumber positionkD = new LoggedTunableNumber("Elevator/positionkD",
      ElevatorConstants.positionD);
  private static final LoggedTunableNumber maxVelocityMetersPerSec = new LoggedTunableNumber(
      "Elevator/MaxVelocityInchesPerSec", ElevatorConstants.maxVelocity);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 = new LoggedTunableNumber(
      "Elevator/MaxAccelerationInchesPerSec2", ElevatorConstants.maxAcceleration);
  private static final LoggedTunableNumber manualVelocInchesPerSec = new LoggedTunableNumber(
      "Elevator/manualVelocityInchesPerSec", ElevatorConstants.manualVelocity);

  /**
   * Elevator auto positions options - pass these into the setElevatorToLocation()
   * function
   */
  public enum elevatorPositions {
    preLoadPosition,
    loaded,
    homePosition,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }



  /**
   * Creates elevator subsystem - this class gives orders to an IO class that
   * executes them
   *
   * <p>
   * This allows quick swapping motor types, PID types, or if an empty elevator IO
   * is passed in
   * it turns off elevator w/o breaking commands
   *
   * @param elevatorIO Class to execute elevator commands
   */
  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    // Configures sys id - this charachterizes the voltage required to move elevator
    // up and down. It also calculates the force of gravity on elevator at various
    // heights allowing accurate stabilization
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(5),
            Seconds.of(10),
            (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // Pulls feedback data from arm IO for logging - this is not used in code,
    // simply for logging

    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);

    // Allows in real time changing of PID & max speed constants IF MAIN CONSTANTS
    // TUNING MODE IS TRUE
    // This does not save the constant - make sure to do this in
    // ElevatorConstants.java
    if (positionkP.hasChanged(hashCode()) || positionkD.hasChanged(hashCode())) {
      elevatorIO.setPositionPID(positionkP.get(), 0.0, positionkD.get());
    }
    if (maxVelocityMetersPerSec.hasChanged(hashCode())
        || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      elevatorIO.setMaxVelocityAcceleration(
          maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get());
    }
  }

  public double getHeight(){
    return elevatorInputs.height;
  }

  /**
   * Sets elevator to preset height location Runs through the inches command
   * before going to IO as
   * to log setpoints
   *
   * @param position Position from ElevatorPositions enum
   */
  public void setElevatorToLocation(elevatorPositions position) {
    switch (position) {
      case preLoadPosition:
        setElevatorHeightInches(ElevatorConstants.preLoadHeight);
        break;
      case loaded:
      setElevatorHeightInches(ElevatorConstants.loadedHeight);
      case homePosition:
        setElevatorHeightInches(ElevatorConstants.homeHeight);
        break;
      case kLevel1:
        setElevatorHeightInches(ElevatorConstants.level1Height);
        break;
      case kLevel2:
        setElevatorHeightInches(ElevatorConstants.level2Height);
        break;
      case kLevel3:
        setElevatorHeightInches(ElevatorConstants.level3Height);
        break;
      case kLevel4:
        setElevatorHeightInches(ElevatorConstants.level4Height);
        break;

      default:
        break;
    }
  }

  public Command manualElevatorUpCommand() {
    return Commands.startEnd(() -> elevatorIO.manualElevatorStart(true), () -> elevatorIO.manualElevatorStop(), this);
  }

  public Command manualElevatorDownCommand() {
    return Commands.startEnd(() -> elevatorIO.manualElevatorStart(false), () -> elevatorIO.manualElevatorStop(), this);
  }

  /**
   * @return Return if elevator is within accaptable error margin of POSITION, NOT
   *         VELOCITY, PID
   *         setpoint
   */
  public boolean atTargetHeight() {
    return elevatorInputs.atTarget;
  }

  public Trigger atTargetHeight = new Trigger(() -> elevatorInputs.atTarget);

  /**
   * Sets elevator to specific height (does not tell it to go up or down a certain
   * amount of height,
   * sets it to certain height)
   *
   * @param inches Height measured from base of elevator (elevator 0, not ground)
   */
  public void setElevatorHeightInches(double inches) {
    Logger.recordOutput("Elevator/HeightSetpoint", inches);
    elevatorIO.setElevatorHeight(inches);
  }

  public void manualElevatorUp() {
    Logger.recordOutput("Elevator/Velocity Setpoint", ElevatorConstants.manualVelocity);
    elevatorIO.setVelocity(ElevatorConstants.manualVelocity);
  }

  public void manualElevatorDown() {
    Logger.recordOutput("Elevator/Velocity Setpoint", -ElevatorConstants.manualVelocity);
    elevatorIO.setVelocity(ElevatorConstants.manualVelocity);
  }

  public void manualElevatorStop() {
    Logger.recordOutput("Elevator/Velocity Setpoint", 0);
    elevatorIO.setVelocity(0);
  }

  /**
   * USE CAUTION - THIS RUNS THE ELEVATOR FAST Returns a command to run a
   * quasistatic test in the
   * specified direction. In this test, the mechanism is gradually sped-up such
   * that the voltage
   * corresponding to acceleration is negligible (hence, “as if static”).
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /**
   * USE CAUTION - THIS RUNS THE ELEVATOR FAST Returns a command to run a dynamic
   * test in the
   * specified direction. In this test, a constant ‘step voltage’ is given to the
   * mechanism, so that
   * the behavior while accelerating can be determined.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Sets elevator to certain voltage, overrides PID loop
   *
   * @param output Voltage, generally output from SysID routine
   */
  private void runVoltage(double output) {
    Logger.recordOutput("Elevator/Voltage Setpoint", output);
    elevatorIO.runCharacterization(output);
  }

  public void disable() {
    Logger.recordOutput("Elevator/Velocity Setpoint", 0);
    setElevatorToLocation(elevatorPositions.homePosition);
    // Waot for elevator to get to target before turning off
    atTargetHeight.onTrue(new SequentialCommandGroup(new InstantCommand(() -> elevatorIO.disableBreak(), this),
        new InstantCommand(() -> runVoltage(0), this)));

  }

  public void enable() {
    elevatorIO.enableBreak();
    setElevatorToLocation(elevatorPositions.preLoadPosition);
  }
}
