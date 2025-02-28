package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.coral.indexer.Indexer;
import frc.robot.subsystems.feedback.DriverFeedback;

public class CoralSystem extends SubsystemBase {
    public final Arm arm;
    public final Elevator elevator;
    private final DriverFeedback feedback;
    private final Indexer indexer;

    private CoralState currentState;
    private boolean autoLoadCoral = false;

    private final LoadCoral loadCoralCommand;

    private final Trigger autoLoadTrigger;

    public CoralSystem(Arm arm, Elevator elevator, Indexer indexer, DriverFeedback feedback) {
        this.arm = arm;
        this.elevator = elevator;
        this.indexer = indexer;
        this.feedback = feedback;

        currentState = CoralState.kStow;

        loadCoralCommand = new LoadCoral(this, feedback);
        autoLoadTrigger = new Trigger(() -> autoLoadCoral == true).and(indexer.getAutoLoadTrigger());

        // If autoLoadCoral is true run "loadCoralCommand" when in range
        autoLoadTrigger.onTrue(
                new SequentialCommandGroup(loadCoralCommand, new InstantCommand(() -> this.autoLoadCoral = false)));
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
                indexer.disableIndexMotor();

                elevator.setElevatorToLocation(elevatorPositions.homePosition);
                arm.setArmToPosition(ArmPositions.homePosition);
            }
            case kL1 -> {
                indexer.disableIndexMotor();

                elevator.setElevatorToLocation(elevatorPositions.kLevel1);
                arm.setArmToPosition(ArmPositions.kLevel1);
            }
            case kL2 -> {
                indexer.disableIndexMotor();

                elevator.setElevatorToLocation(elevatorPositions.kLevel2);
                arm.setArmToPosition(ArmPositions.kLevel2);
            }
            case kL3 -> {
                indexer.disableIndexMotor();

                elevator.setElevatorToLocation(elevatorPositions.kLevel3);
                arm.setArmToPosition(ArmPositions.kLevel3);
            }
            case kL4 -> {
                indexer.disableIndexMotor();

                elevator.setElevatorToLocation(elevatorPositions.kLevel4);
                arm.setArmToPosition(ArmPositions.kLevel4);
            }
            case kPreLoad -> {
                elevator.setElevatorToLocation(elevatorPositions.preLoadPosition);
                arm.setArmToPosition(ArmPositions.loadPosition);
                indexer.enableIndexMotor();

                if (CoralConstants.autoLoadEnabled) {
                    autoLoadCoral = true;
                }
            }
            default -> throw new AssertionError();
        }
    }
    public Trigger getAutoLoadTrigger() {
        return autoLoadTrigger;
    }

    public void disable() {
        elevator.disable();
        arm.disable();
        indexer.disableIndexMotor();
    }

    public void enable() {
        elevator.enable();
        arm.enable();
        indexer.disableIndexMotor();
    }
}
