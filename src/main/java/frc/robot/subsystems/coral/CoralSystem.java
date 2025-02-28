package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.coral.LoadCoral;
import frc.robot.subsystems.coral.CoralConstants.CoralState;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.arm.Arm.ArmPositions;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.elevator.Elevator.elevatorPositions;
import frc.robot.subsystems.feedback.DriverFeedback;

public class CoralSystem extends SubsystemBase {
    public final Arm arm;
    public final Elevator elevator;
    private final CANrange canRange;
    private final DriverFeedback feedback;

    private CoralState currentState;
    private boolean autoLoadCoral = false;
    Trigger autoLoadTrigger;

    private final LoadCoral loadCoralCommand;

    public CoralSystem(Arm arm, Elevator elevator, DriverFeedback feedback) {
        this.arm = arm;
        this.elevator = elevator;
        this.canRange = new CANrange(CoralConstants.canRangeID);
        this.feedback = feedback;

        currentState = CoralState.kStow;

        loadCoralCommand = new LoadCoral(this, feedback);

        // If autoLoadCoral is true run "loadCoralCommand" when in range
        autoLoadTrigger = new Trigger(this::triggerCoralAutoLoad);
        autoLoadTrigger.onTrue(
                new SequentialCommandGroup(loadCoralCommand, new InstantCommand(() -> this.autoLoadCoral = false)));
    }

    private boolean triggerCoralAutoLoad() {
        return canRange.getIsDetected().getValue() && autoLoadCoral;
    }

    public void setCoralState(CoralState state) {
        if (state == currentState) {
            System.err.println("Coral Subsystem set to current state");
            return;
        }
        Logger.recordOutput("Coral/state", state);
        currentState = state;

        switch (state) {
            case kStow -> {
                elevator.setElevatorToLocation(elevatorPositions.homePosition);
                arm.setArmToAngle(ArmPositions.homePosition);
            }
            case kL1 -> {
                elevator.setElevatorToLocation(elevatorPositions.kLevel1);
                arm.setArmToAngle(ArmPositions.kLevel1);
            }
            case kL2 -> {
                elevator.setElevatorToLocation(elevatorPositions.kLevel2);
                arm.setArmToAngle(ArmPositions.kLevel2);
            }
            case kL3 -> {
                elevator.setElevatorToLocation(elevatorPositions.kLevel3);
                arm.setArmToAngle(ArmPositions.kLevel3);
            }
            case kL4 -> {
                elevator.setElevatorToLocation(elevatorPositions.kLevel4);
                arm.setArmToAngle(ArmPositions.kLevel4);
            }
            case kPreLoad -> {
                elevator.setElevatorToLocation(elevatorPositions.preLoadPosition);
                arm.setArmToAngle(ArmPositions.loadPosition);

                if (CoralConstants.autoLoadEnabled) {
                    autoLoadCoral = true;
                }
            }
            default -> throw new AssertionError();
        }
    }

    public void disable() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disable'");
    }

    public void enable() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enable'");
    }
}
