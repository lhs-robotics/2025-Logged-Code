package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  final ArmIO armIO;
  private final SysIdRoutine sysId;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  // Tunable Numbers
  private static final LoggedTunableNumber positionkP =
      new LoggedTunableNumber("Arm/positionkP", ArmConstants.positionP);
  private static final LoggedTunableNumber positionkD =
      new LoggedTunableNumber("Arm/positionkD", ArmConstants.positionD);
  private static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Arm/MaxVelocityInchesPerSec", ArmConstants.maxVelocity);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Arm/MaxAccelerationInchesPerSec2", ArmConstants.maxAcceleration);
  private static final LoggedTunableNumber manualVelocInchesPerSec =
      new LoggedTunableNumber("Arm/manualVelocityInchesPerSec", ArmConstants.manualVelocity);

  public enum ArmPositions {
    loadPosition,
    homePosition,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  public Arm(ArmIO armIO) {
    this.armIO = armIO;

    sysId =
        new SysIdRoutine(
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
    armIO.updateInputs(armInputs);
    Logger.processInputs("Arm", armInputs);

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
   * Sets Arm to preset height location Runs through the inches command before going to IO as to log
   * setpoints
   *
   * @param position
   */
  public void setArmToAngle(ArmPositions position) {
    switch (position) {
      case loadPosition:
        setArmAngleInches(ArmConstants.loadAngle);
        break;
      case homePosition:
        setArmAngleInches(ArmConstants.homeAngle);
        break;
      case kLevel1:
        setArmAngleInches(ArmConstants.level1Angle);
        break;
      case kLevel2:
        setArmAngleInches(ArmConstants.level2Angle);
        break;
      case kLevel3:
        setArmAngleInches(ArmConstants.level3Angle);
        break;
      case kLevel4:
        setArmAngleInches(ArmConstants.level4Angle);
        break;

      default:
        break;
    }
  }

  public boolean atTargetAngle() {
    return armInputs.atTarget;
  }

  public void setArmAngleInches(double degrees) {
    Logger.recordOutput("Arm/AngleSetpoint", degrees);
    armIO.setArmAngleDegrees(degrees);
  }

  public void manualArmUp() {
    Logger.recordOutput("Arm/Velocity Setpoint", ArmConstants.manualVelocity);
    armIO.setVelocity(ArmConstants.manualVelocity);
  }

  public void manualArmDown() {
    Logger.recordOutput("Arm/Velocity Setpoint", -ArmConstants.manualVelocity);
    armIO.setVelocity(ArmConstants.manualVelocity);
  }

  public void manualArmStop() {
    Logger.recordOutput("Arm/Velocity Setpoint", 0);
    armIO.setVelocity(0);
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
}
