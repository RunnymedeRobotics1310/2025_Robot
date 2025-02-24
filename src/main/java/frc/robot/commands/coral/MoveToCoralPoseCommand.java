package frc.robot.commands.coral;

import frc.robot.Constants.CoralConstants.CoralPose;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class MoveToCoralPoseCommand extends LoggingCommand {

  private final CoralSubsystem coralSubsystem;
  private final CoralPose pose;

  boolean atElevatorHeight = false;
  boolean atArmAngle = false;

  public MoveToCoralPoseCommand(CoralPose pose, CoralSubsystem coralSubsystem) {

    this.coralSubsystem = coralSubsystem;
    this.pose = pose;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {

    logCommandStart();

    atElevatorHeight = false;
    atArmAngle = false;
  }

  @Override
  public void execute() {

    atElevatorHeight = coralSubsystem.moveElevatorToHeight(pose.elevatorHeight);
    atArmAngle = coralSubsystem.moveArmToAngle(pose.armAngle);
  }

  @Override
  public boolean isFinished() {

    return atElevatorHeight && atArmAngle;
  }
}
