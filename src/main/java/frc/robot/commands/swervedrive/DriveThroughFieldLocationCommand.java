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
  private final boolean driveThrough;
  private final double toleranceM;
  private final double decelDistance;

  public DriveThroughFieldLocationCommand(
      SwerveSubsystem swerve, FieldLocation location, double maxSpeed, boolean driveThrough) {
    this(swerve, location, maxSpeed, driveThrough, 0.20, 1.2);
  }

  public DriveThroughFieldLocationCommand(
      SwerveSubsystem swerve,
      FieldLocation location,
      double maxSpeed,
      boolean driveThrough,
      double toleranceM,
      double decelDistance) {
    this.swerve = swerve;
    this.driveThrough = driveThrough;
    this.toleranceM = toleranceM;
    this.decelDistance = decelDistance;

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

    //    double hypot = Math.hypot(xDif, yDif);
    //    double speed = swerve.computeTranslateVelocity(hypot, 1, 0.02);
    //    double xSpeed = Math.cos(speed / maxSpeed);
    //    double ySpeed = Math.sin(speed / maxSpeed);

    Translation2d totalDif = new Translation2d(xDif, yDif);
    Translation2d totalSpeed =
        swerve.computeTranslateVelocity2024(totalDif, maxSpeed, 0.02, driveThrough, decelDistance);
    double xSpeed = totalSpeed.getX();
    double ySpeed = totalSpeed.getY();

    swerve.driveFieldOriented(xSpeed, ySpeed, swerve.computeOmega(targetHeadingDeg));
  }

  @Override
  public boolean isFinished() {
    return (SwerveUtils.isCloseEnough(
        swerve.getPose().getTranslation(), allianceLocation.getTranslation(), toleranceM));
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
  }
}
