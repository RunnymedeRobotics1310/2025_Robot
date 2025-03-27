package frc.robot.commands.swervedrive;

import static frc.robot.Constants.AutoConstants.*;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveThroughFieldLocationCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final Pose2d allianceLocation;
  private final double maxSpeed;
  private final double targetHeadingDeg;
  private final double decelZoneM;
  private final double toleranceM;
  private final double toleranceDeg;

  public DriveThroughFieldLocationCommand(
      SwerveSubsystem swerve,
      FieldLocation location,
      double maxSpeed,
      double decelZoneM,
      double toleranceM,
      double toleranceDeg) {
    this.swerve = swerve;
    this.decelZoneM = decelZoneM;
    this.toleranceM = toleranceM;
    this.toleranceDeg = toleranceDeg;

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      allianceLocation = RunnymedeUtils.getRedAlliancePose(location.pose);
    } else {
      allianceLocation = location.pose;
    }
    this.maxSpeed = maxSpeed;
    this.targetHeadingDeg =
        SwerveUtils.normalizeDegrees(allianceLocation.getRotation().getDegrees());

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    double xDif = allianceLocation.getX() - currentPose.getX();
    double yDif = allianceLocation.getY() - currentPose.getY();

    Translation2d totalDif = new Translation2d(xDif, yDif);
    Translation2d totalSpeed =
        swerve.computeTranslateVelocity2024(totalDif, maxSpeed, toleranceM, decelZoneM);
    double xSpeed = totalSpeed.getX();
    double ySpeed = totalSpeed.getY();

    swerve.driveFieldOriented(xSpeed, ySpeed, swerve.computeOmega(targetHeadingDeg));
  }

  @Override
  public boolean isFinished() {
    return SwerveUtils.isCloseEnough(
            swerve.getPose().getTranslation(), allianceLocation.getTranslation(), 0.20)
        && Math.abs(swerve.getYaw() - targetHeadingDeg) >= toleranceDeg;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
  }
}
