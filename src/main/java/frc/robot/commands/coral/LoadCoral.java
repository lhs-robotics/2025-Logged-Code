package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.Arm.ArmIntakeCommand;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.elevator.Elevator.elevatorPositions;
import frc.robot.subsystems.feedback.DriverFeedback;
import frc.robot.subsystems.feedback.FeedbackConstants.FeedbackType;

public class LoadCoral extends SequentialCommandGroup {
    public LoadCoral(CoralSystem coralSys, DriverFeedback feedback) {
        addCommands(
            new ParallelCommandGroup(
                new ArmIntakeCommand(coralSys),
                new InstantCommand(() -> coralSys.elevator.setElevatorToLocation(elevatorPositions.loaded))) 
                .andThen(new InstantCommand(() -> feedback.giveFeedback(FeedbackType.loaded)))); // Run arm motor until elevator is down
    }
}
