package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.coral.elevator.Elevator.elevatorPositions;

public class ReleaseCoral extends SequentialCommandGroup {
    public ReleaseCoral(CoralSystem coralSys) {
        addCommands(coralSys.arm.releaseCoral(), // Delay built into release coral motor command, causes this to delay
                                                 // appropiatley
                new InstantCommand(() -> coralSys.elevator.setElevatorToLocation(elevatorPositions.preLoadPosition)));
    }
}
