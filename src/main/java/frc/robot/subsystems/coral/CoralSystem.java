package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ArmIntakeCommand;
import frc.robot.subsystems.coral.CoralConstants.CoralState;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.arm.Arm.ArmPositions;
import frc.robot.subsystems.coral.elevator.Elevator;
import frc.robot.subsystems.coral.elevator.Elevator.elevatorPositions;

public class CoralSystem extends SubsystemBase {
    private final Arm arm;
    private final Elevator elevator;
    private final CANrange canRange;
    private CoralState currentState;
    private boolean autoLoadCoral = false;
    ArmIntakeCommand armIntakeCommand;
    Trigger autoLoadTrigger;

    public CoralSystem(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;
        this.canRange = new CANrange(CoralConstants.canRangeID);

        currentState = CoralState.kStow;

        // When 
        armIntakeCommand = new ArmIntakeCommand(arm, elevator);
        autoLoadTrigger = new Trigger(this::triggerCoralAutoLoad);
        autoLoadTrigger.onTrue(armIntakeCommand);
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

    public Command loadCoral () {
        return 
    }
}
