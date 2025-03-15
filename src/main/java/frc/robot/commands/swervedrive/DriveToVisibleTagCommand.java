package frc.robot.commands.swervedrive;

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

  public DriveToVisibleTagCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, boolean isLeftBranch) {
    this.swerve = swerve;
    this.vision = vision;
    this.isLeftBranch = isLeftBranch;
    addRequirements(swerve, vision);
  }

  @Override
  public void execute() {

    // capture tag if we don't have one
    if (tagId == -1) {
      tagId = (int) vision.getVisibleTargetTagId(isLeftBranch);
      if (tagId == -1) {
        noDataCount++;
        return;
      } else {
        noDataCount = 0;
        log("Captured tag " + tagId + " on the " + (isLeftBranch ? "left" : "right") + " branch");
      }
    }

    // get offset
    vision.setTargetTagId(tagId);
    final double tX;
    if (vision.isTagInView(tagId, isLeftBranch)) {
      noDataCount = 0;
      tX = vision.angleToTarget(isLeftBranch);
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
      vX = 0.25;
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
    vision.clearTargetTagId();
    swerve.stop();
  }
}
