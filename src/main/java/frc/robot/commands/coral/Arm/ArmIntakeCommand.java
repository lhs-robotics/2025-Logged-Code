package frc.robot.commands.coral.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSystem;

public class ArmIntakeCommand extends Command {
  private final CoralSystem coralSys;

  public ArmIntakeCommand(CoralSystem coralSys) {
    this.coralSys = coralSys;
    addRequirements(coralSys);
  }

  @Override
  public void initialize() {
    coralSys.arm.endAffectorIntakeEnable();
  }

  @Override
  public boolean isFinished() {
    return coralSys.elevator.atTargetHeight();
  }

  @Override
  public void end(boolean interrupted) {
    coralSys.arm.endAffectorIntakeDisable();
  }

}
