package frc.robot.commands.coral.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.arm.Arm;
import frc.robot.subsystems.coral.elevator.Elevator;

public class ArmIntakeCommand extends Command {
    Arm arm;
    Elevator elevator;
    
    public ArmIntakeCommand (Arm arm, Elevator elevator) {
      this.arm = arm;
      this.elevator = elevator;
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      arm.endAffectorIntakeEnable();
    }

    @Override
    public boolean isFinished() {
      return elevator.atTargetHeight();
    }

    @Override
    public void end(boolean interrupted) {
      arm.endAffectorIntakeDisable();
    }

  }
