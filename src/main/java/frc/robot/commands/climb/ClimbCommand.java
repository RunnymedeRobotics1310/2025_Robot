package frc.robot.commands.climb;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.BooleanSupplier;

public class ClimbCommand extends LoggingCommand {

  private final BooleanSupplier deployClimb;
  private final ClimbSubsystem climbSubsystem;

  public ClimbCommand(BooleanSupplier deployClimb, ClimbSubsystem climbSubsystem) {

    this.deployClimb = deployClimb;
    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {

    logCommandStart();

    if (deployClimb.getAsBoolean()) {
      climbSubsystem.setClimbDeployed(true);
    } else {
      climbSubsystem.setClimbDeployed(false);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
  }
}
