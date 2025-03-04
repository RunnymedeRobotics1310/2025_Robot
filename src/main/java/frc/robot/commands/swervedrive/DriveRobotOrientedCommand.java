package frc.robot.commands.swervedrive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveRobotOrientedCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final double x;
  private final double y;
  private final double angle;

  public DriveRobotOrientedCommand(SwerveSubsystem swerve, double x, double y, double angle) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.angle = angle;
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    return;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    logCommandEnd(interrupted);
  }
}
