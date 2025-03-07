package frc.robot.commands.swervedrive;

import static frc.robot.Constants.AutoConstants.FieldLocation;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToFieldLocationCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final Pose2d location;
  private Pose2d allianceLocation;
  private double targetHeadingDeg;

  public DriveToFieldLocationCommand(SwerveSubsystem swerve, FieldLocation location) {
    this.swerve = swerve;
    this.location = location.pose;
  }

  @Override
  public void initialize() {
    logCommandStart();

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      this.allianceLocation = RunnymedeUtils.getRedAlliancePose(location);
    } else {
      this.allianceLocation = location;
    }
    this.targetHeadingDeg = SwerveUtils.normalizeDegrees(location.getRotation().getDegrees());

    log("Pose: " + swerve.getPose());
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    double xDif = allianceLocation.getX() - currentPose.getX();
    double yDif = allianceLocation.getY() - currentPose.getY();

    double angleDif =
        SwerveUtils.normalizeDegrees(targetHeadingDeg - currentPose.getRotation().getDegrees());
    //        log("Xdif: " + xDif + " Ydif: " + yDif + " ºdif: " + angleDif);

    swerve.driveFieldOriented(
        swerve.computeTranslateVelocity(xDif, 0.02),
        swerve.computeTranslateVelocity(yDif, 0.02),
        swerve.computeOmega(targetHeadingDeg));
  }

  @Override
  public boolean isFinished() {
    //        return (SwerveUtils.isCloseEnough(
    //                swerve.getPose().getTranslation(), location.pose.getTranslation(), 0.05)
    //                && SwerveUtils.isCloseEnough(swerve.getPose().getRotation().getDegrees(),
    // targetHeadingDeg, 10));
    boolean done =
        (SwerveUtils.isCloseEnough(
                swerve.getPose().getTranslation(), allianceLocation.getTranslation(), 0.05)
            && SwerveUtils.isCloseEnough(swerve.getYaw(), targetHeadingDeg, 10));
    if (done) {
      System.out.println(
          "REACHED DESTINATION: x["
              + swerve.getPose().getX()
              + "] y["
              + swerve.getPose().getY()
              + "], deg["
              + swerve.getPose().getRotation().getDegrees()
              + "]");
    }
    return done;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
  }
}
