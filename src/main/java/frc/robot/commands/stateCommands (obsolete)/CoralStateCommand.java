package frc.robot.commands.stateCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;

public class CoralStateCommand extends InstantCommand {
    // Parallel:
    // 1. Set climb setpoint 0
    // 2. Set algae setpoint 0
    // 3. Set elevator arm to preload position
    
}
