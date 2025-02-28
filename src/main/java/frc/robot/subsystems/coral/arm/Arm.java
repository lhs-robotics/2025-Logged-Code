package frc.robot.subsystems.coral.arm;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
  final ArmIO armIO;
  private final SysIdRoutine sysId;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  // Tunable Numbers - Settable in Advantage Scope when tuning mode is enabled in
  // main constants
  // THESE ARE NOT SAVED - YOU MUST ENTER THEM INTO ARMCONSTANTS.JAVA
  private static final LoggedTunableNumber positionkP = new LoggedTunableNumber("Arm/positionkP",
      ArmConstants.positionP);
  private static final LoggedTunableNumber positionkD = new LoggedTunableNumber("Arm/positionkD",
      ArmConstants.positionD);
  private static final LoggedTunableNumber maxVelocityMetersPerSec = new LoggedTunableNumber(
      "Arm/MaxVelocityInchesPerSec", ArmConstants.maxVelocity);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 = new LoggedTunableNumber(
      "Arm/MaxAccelerationInchesPerSec2", ArmConstants.maxAcceleration);
  private static final LoggedTunableNumber manualVelocInchesPerSec = new LoggedTunableNumber(
      "Arm/manualVelocityInchesPerSec", ArmConstants.manualVelocity);

  /** Arm auto angling options - pass these into the setArmToAngle() function */
  public enum ArmPositions {
    loadPosition,
    homePosition,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  /**
   * Creates arm subsystem - this class gives orders to an IO class that executes
   * them
   *
   * <p>
   * This allows quick swapping motor types, PID types, or if an empty arm IO is
   * passed in it
   * turns off elevator w/o breaking commands
   *
   * @param elevatorIO Class to execute arm commands
   */
  public Arm(ArmIO armIO) {
    this.armIO = armIO;

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // Pulls feedback data from arm IO for logging - this is not used in code,
    // simply for logging
    armIO.updateInputs(armInputs);
    Logger.processInputs("Arm", armInputs);

    // Allows in real time changing of PID & max speed constants IF MAIN CONSTANTS
    // TUNING MODE IS TRUE
    // This does not save the constant - make sure to do this in ArmConstants.java
    if (positionkP.hasChanged(hashCode()) || positionkD.hasChanged(hashCode())) {
      armIO.setPositionPID(positionkP.get(), 0.0, positionkD.get());
    }
    if (maxVelocityMetersPerSec.hasChanged(hashCode())
        || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      armIO.setMaxVelocityAcceleration(
          maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get());
    }
  }

  /**
   * Sets Arm to preset angle; Runs through the set angle command before going to
   * IO as to log
   * setpoints
   *
   * @param position
   */
  public void setArmToPosition(ArmPositions position) {
    switch (position) {
      case loadPosition -> setArmAngleDegrees(ArmConstants.loadAngle);
      case homePosition -> setArmAngleDegrees(ArmConstants.homeAngle);
      case kLevel1 -> setArmAngleDegrees(ArmConstants.level1Angle);
      case kLevel2 -> setArmAngleDegrees(ArmConstants.level2Angle);
      case kLevel3 -> setArmAngleDegrees(ArmConstants.level3Angle);
      case kLevel4 -> setArmAngleDegrees(ArmConstants.level4Angle);

      default -> {
      }
    }
  }

  /**
   * @return Returns if arm is within accaptable error margin of PID setpoint (FOR
   *         ANGLE)
   */
  public boolean atTargetAngle() {
    return armInputs.atTarget;
  }

  /**
   * Changes the arm position to an arbitrary degree value measured from arm
   * straight up
   *
   * @param degrees
   */
  public void setArmAngleDegrees(double degrees) {
    Logger.recordOutput("Arm/AngleSetpoint", degrees);
    armIO.setArmAngleDegrees(degrees);
  }

  /** Sets the velocity of arm to arbitrarily move up */
  public void manualArmUp() {
    Logger.recordOutput("Arm/Velocity Setpoint", ArmConstants.manualVelocity);
    armIO.setVelocity(ArmConstants.manualVelocity);
  }

  /** Sets the velocity of arm to abritrarilly move down */
  public void manualArmDown() {
    Logger.recordOutput("Arm/Velocity Setpoint", -ArmConstants.manualVelocity);
    armIO.setVelocity(ArmConstants.manualVelocity);
  }

  /** Sets the velocity of arm to 0 */
  public void manualArmStop() {
    Logger.recordOutput("Arm/Velocity Setpoint", 0);
    armIO.setVelocity(0);
  }


  // Sepereate commands for easy use in auto load routine
  public void endAffectorIntakeEnable() {
    Logger.recordOutput("Arm/End Affector Speed Setpoint", ArmConstants.coralIntakeSpeed);
    armIO.setEndAffectorSpeed(ArmConstants.coralIntakeSpeed);
  }
  public void endAffectorIntakeDisable() {
    Logger.recordOutput("Arm/End Affector Speed Setpoint", 0);
    armIO.setEndAffectorSpeed(0);
  }

  public Command releaseCoral() {
    return new SequentialCommandGroup(new InstantCommand(this::releaseCoralBegin), new WaitCommand(ArmConstants.coralReleaseTimeSecs), new InstantCommand(this::endAffectorIntakeDisable));
  }

  private void releaseCoralBegin() {
    Logger.recordOutput("Arm/End Affector Speed Setpoint", ArmConstants.coralReleaseSpeed);
    armIO.setEndAffectorSpeed(ArmConstants.coralReleaseSpeed);
  }


  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  private void runCharacterization(double output) {
    armIO.runCharacterization(output);
  }

public void disable() {
    setArmToPosition(ArmPositions.homePosition);
    endAffectorIntakeDisable();
    armIO.disableEndAffectorBrake();
}

public void enable() {
  setArmToPosition(ArmPositions.loadPosition);
  armIO.enableEndAffectorBrake();
}

  
}
