package frc.robot.commands.coral.elevator;

import frc.robot.Constants.CoralConstants.ElevatorHeight;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class MoveElevatorToHeightCommand extends LoggingCommand {

  private final ElevatorHeight elevatorHeight;
  private final CoralSubsystem coralSubsystem;

  public MoveElevatorToHeightCommand(ElevatorHeight elevatorHeight, CoralSubsystem coralSubsystem) {

    this.elevatorHeight = elevatorHeight;
    this.coralSubsystem = coralSubsystem;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    coralSubsystem.moveElevatorToHeight(elevatorHeight);
  }

  @Override
  public boolean isFinished() {
    return coralSubsystem.isAtElevatorHeight(elevatorHeight);
  }

  @Override
  public void end(boolean interrupted) {
    coralSubsystem.setElevatorSpeed(0);
    logCommandEnd(interrupted);
  }
}
