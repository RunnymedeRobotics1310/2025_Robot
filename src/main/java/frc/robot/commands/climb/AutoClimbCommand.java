package frc.robot.commands.climb;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class AutoClimbCommand extends LoggingCommand {

  private final ClimbSubsystem climbSubsystem;

  public AutoClimbCommand(ClimbSubsystem climbSubsystem) {

    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {

    logCommandStart();
  }

  @Override
  public void execute() {

    if (climbSubsystem.isCageInPosition()) {
      climbSubsystem.setClimbDeployed(true);
    }
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isCageInPosition();
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
  }
}
