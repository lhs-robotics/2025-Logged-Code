package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;

public class OperatorCommands {


    private OperatorCommands() {}

    public static Command Level4(Elevator elevator, Arm arm) {
        return Commands.run(
                () -> {
                    elevator.setElevatorHeightInches(ElevatorConstants.level4Height);
                    arm.setArmAngleDegrees(ArmConstants.level4Angle);
                }, elevator, arm);
    }

    public static Command Level3(Elevator elevator, Arm arm) {
        return Commands.run(
                () -> {
                    elevator.setElevatorHeightInches(ElevatorConstants.level3Height);
                    arm.setArmAngleDegrees(ArmConstants.level3Angle);
                }, elevator, arm);
    }

    public static Command Level2(Elevator elevator, Arm arm) {
        return Commands.run(
                () -> {
                    elevator.setElevatorHeightInches(ElevatorConstants.level2Height);
                    arm.setArmAngleDegrees(ArmConstants.level2Angle);
                }, elevator, arm);
    }

    public static Command Level1(Elevator elevator, Arm arm) {
        return Commands.run(
                () -> {
                    elevator.setElevatorHeightInches(ElevatorConstants.level1Height);
                    arm.setArmAngleDegrees(ArmConstants.level1Angle);
                }, elevator, arm);
    }


    public static Command ElevatorManualUp(Elevator elevator){
        return Commands.run(() -> { elevator.manualElevatorUp();
        }, elevator);

    }

    public static Command ElevatorManualDown(Elevator elevator){
        return Commands.run(() -> { elevator.manualElevatorDown();
        }, elevator);

    }

    public static Command ArmManualUp(Arm arm){
        return Commands.run(() -> { arm.manualArmUp();
        }, arm);
    }

    public static Command ArmManualDown(Arm arm){
        return Commands.run(() -> { arm.manualArmDown();
        }, arm);
    }

    public static Command ClimbUp(Climb climb){
        return Commands.run(() -> { climb.climbUp();
        }, climb);
    }

    public static Command ClimbDown(Climb climb){
        return Commands.run(() -> { climb.climbDown();
        }, climb);
    }
   
    public static Command ClimbOff(Climb climb){
        return Commands.run(() -> { climb.turnOff();
        }, climb);
    }
}
