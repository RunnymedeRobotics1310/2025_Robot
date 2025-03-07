package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveRobotOrientedCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final double x;
  private final double y;
  private final double heading;
  private double allianceHeading;

  public DriveRobotOrientedCommand(SwerveSubsystem swerve, double x, double y, double heading) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.heading = heading;
  }

  @Override
  public void initialize() {
    logCommandStart();

    double headingOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      headingOffset = 180;
    }

    allianceHeading = heading + headingOffset;
  }

  @Override
  public void execute() {
    swerve.driveRobotOriented(x, y, swerve.computeOmega(allianceHeading));
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
