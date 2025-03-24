package frc.robot.commands.swervedrive;

import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants.TAGS;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToReefTagCommand extends LoggingCommand {

  private static final int MAX_NO_DATA_COUNT_CYCLES = 50;

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;
  private final FieldLocation location;

  private int tagId = -1;
  private int noDataCount = 0;
  private boolean isLeftBranch = true;

  public DriveToReefTagCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, FieldLocation location) {
    this.swerve = swerve;
    this.vision = vision;
    this.location = location;
    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    logCommandStart();

    noDataCount = 0;
    isLeftBranch = location.isLeftSide;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      tagId = location.redTagId;
    } else {
      tagId = location.blueTagId;
    }
  }

  @Override
  public void execute() {

    // get offset
    final double tX;
    if (vision.isTagInView(tagId, isLeftBranch)) {
      noDataCount = 0;
      tX = vision.angleToTarget(tagId, isLeftBranch);
    } else {
      noDataCount++;

      double theta = TAGS.getTagById(tagId).pose.getRotation().getDegrees();
      double omega = swerve.computeOmega(theta);

      swerve.driveRobotOriented(0, 0, omega);
      return;
    }

    // drive to tag
    final double vX;
    final double vY;
    if (Math.abs(tX) > 20) {
      vX = 0;
    } else {
      vX = 0.301;
    }
    vY = 0.02 * tX;

    double theta = TAGS.getTagById(tagId).pose.getRotation().getDegrees();
    double omega = swerve.computeOmega(theta);

    swerve.driveRobotOriented(vX, vY, omega);
  }

  @Override
  public boolean isFinished() {
    double theta = TAGS.getTagById(tagId).pose.getRotation().getDegrees();

    if (noDataCount > MAX_NO_DATA_COUNT_CYCLES && Math.abs(swerve.getYaw() - theta) < 5) {
      log("Finishing - no vision data for " + noDataCount + " cycles");
      return true;
    }

    double distanceToReef = swerve.getUltrasonicDistanceM();

    if (distanceToReef < 0.03) {
      log("Finishing - " + distanceToReef + " from reef");
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    noDataCount = 0;
    swerve.stop();
  }
}
