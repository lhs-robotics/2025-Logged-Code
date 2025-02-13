package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
    final ElevatorIO elevatorIO;
    private final SysIdRoutine sysId;

    public enum elevatorPositions {
        loadPosition,
        homePosition,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }


    /**
     * Sets elevator to preset height location
     * Runs through the inches command before going to IO as to log setpoints
     * @param position
     */
    public void setElevatorToLocation(elevatorPositions position) {
        switch (position) {
            case loadPosition:
                setElevatorHeightInches(ElevatorConstants.loadHeight);
                break;
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

    public void setElevatorHeightInches(double inches){
        Logger.recordOutput("Elevator/HeightSetpoint", inches);
        elevatorIO.setElevatorHeightInches(inches);
    }

    public void manualElevatorUp(){
        Logger.recordOutput("Elevator/Velocity Setpoint", ElevatorConstants.manualVelocity);
        elevatorIO.setVelocity(ElevatorConstants.manualVelocity);
    }

    public void manualElevatorDown(){
        Logger.recordOutput("Elevator/Velocity Setpoint", -ElevatorConstants.manualVelocity);
        elevatorIO.setVelocity(ElevatorConstants.manualVelocity);
    }

    public void manualElevatorStop(){
        Logger.recordOutput("Elevator/Velocity Setpoint", 0);
        elevatorIO.setVelocity(0);
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

    private void runCharacterization(double output){
        elevatorIO.runCharacterization(output);
    }
}
