package frc.robot.commands.swervedrive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveInlineWithTagCommand extends LoggingCommand {

  private SwerveSubsystem swerve;
  private LimelightVisionSubsystem vision;

  public DriveInlineWithTagCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    double tX = vision.angleToTarget();

    double speed = 0.1 * Math.signum(tX);
    swerve.driveRobotOriented(0, speed, swerve.computeOmega(180));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(vision.angleToTarget()) < 2;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
  }
}
