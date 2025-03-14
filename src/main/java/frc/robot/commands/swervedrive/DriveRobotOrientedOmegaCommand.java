package frc.robot.commands.swervedrive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveRobotOrientedOmegaCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final double x;
  private final double y;
  private final double omega;

  public DriveRobotOrientedOmegaCommand(SwerveSubsystem swerve, double x, double y, double omega) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.omega = omega;
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    swerve.driveRobotOriented(x, y, omega);
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
