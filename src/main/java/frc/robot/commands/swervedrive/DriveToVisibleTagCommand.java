package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants.TAGS;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToVisibleTagCommand extends LoggingCommand {

  private static final int MAX_NO_DATA_COUNT_CYCLES = 20;

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;
  private final boolean isLeftBranch;

  private int tagId = -1;
  private int noDataCount = 0;

  private double lastDistance = 5000;
  private double lastDistanceChangeTime = Timer.getFPGATimestamp();

  public DriveToVisibleTagCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, boolean isLeftBranch) {
    this.swerve = swerve;
    this.vision = vision;
    this.isLeftBranch = isLeftBranch;
    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    logCommandStart();
    tagId = -1;
    noDataCount = 0;
    lastDistance = 5000;
    lastDistanceChangeTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

    // capture tag if we don't have one
    if (tagId == -1) {
      tagId = (int) vision.getVisibleTargetTagId(isLeftBranch);
      if (tagId == -1) {
        tagId = (int) vision.getVisibleTargetTagId(!isLeftBranch);
        if (tagId == -1) {
          noDataCount++;
          return;
        }
      }
      if (tagId != -1) {
        noDataCount = 0;
        log("Captured tag " + tagId + " on the " + (isLeftBranch ? "left" : "right") + " branch");
      }
    }

    // get offset
    final double tX;
    if (vision.isTagInView(tagId, isLeftBranch)) {
      noDataCount = 0;
      tX = vision.angleToTarget(tagId, isLeftBranch);
    } else if (vision.isTagInView(tagId, !isLeftBranch)) {
      noDataCount = 0;
      if (isLeftBranch) {
        tX = 20;
      } else {
        tX = -20;
      }
    } else {
      noDataCount++;
      return;
    }

    // drive to tag
    final double vX;
    final double vY;
    if (Math.abs(tX) > 20) {
      vX = 0;
    } else {
      vX = 0.35;
    }
    vY = 0.02 * tX;

    double theta = TAGS.getTagById(tagId).pose.getRotation().getDegrees();
    double omega = swerve.computeOmega(theta);

    swerve.driveRobotOriented(vX, vY, omega);
  }

  @Override
  public boolean isFinished() {

    if (noDataCount > MAX_NO_DATA_COUNT_CYCLES) {
      log("Finishing - no vision data for " + noDataCount + " cycles");
      return true;
    }

    double distanceToTag = vision.distanceTagToFrontBumper(tagId, isLeftBranch);
    double currentTime = Timer.getFPGATimestamp();

    if (Math.abs(Math.round((distanceToTag - lastDistance) * 100d) / 100d) > 0) {
      lastDistanceChangeTime = currentTime;
    }
    double elaspedTimeSinceUpdate = currentTime - lastDistanceChangeTime;

    // Update last distance
    lastDistance = distanceToTag;

    // Not checking tx because if we're this close, we can't move left/right anyways.  Check > -1 as
    // it will return that if there's no tag data.  Also have a 2 second timeout if we aren't moving
    // closer to the tag
    if (distanceToTag < 0.04 && distanceToTag > -1 || elaspedTimeSinceUpdate > 2) {
      log(
          "Finishing: DistanceToTag[ "
              + distanceToTag
              + "], TimeSinceLastDistanceChange: ["
              + elaspedTimeSinceUpdate
              + "]");
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
