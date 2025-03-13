package frc.robot.commands.swervedrive;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToScorePositionCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem visionSubsystem;
  private boolean isLeftBranch;
  private final Constants.AutoConstants.FieldLocation fieldLocation;

  private Pose2d tagPose;
  private double targetHeadingDeg;
  private Pose2d initialRobotPose;
  private Pose2d targetPose;
  private int tagId = 0;
  private boolean noTagsAbort = false;
  private boolean atElevatorHeight = false;
  private boolean atArmAngle = false;

  public static final double OFFSET_FROM_TAG_FOR_SCORING = 0.20;
  public static final double OFFSET_FROM_TAG_ROBOT_HALF_LENGTH = 0.57;

  /**
   * Drive to a scoring position based on a vision target tag. If no tag is specified, it'll use the
   * currently visible closest one.
   *
   * <p>NOTE: This is the command used @ Centennial. It is now considered legacy and to be replaced
   * by better options. Left here as it did work well for scoring one on Right6.
   *
   * @param swerve The swerve subsystem
   * @param visionSubsystem The vision subsystem
   * @param fieldLocation The field location to drive to
   * @param isLeftBranch Should this target the left branch or right?
   */
  public DriveToScorePositionCommand(
      SwerveSubsystem swerve,
      LimelightVisionSubsystem visionSubsystem,
      Constants.AutoConstants.FieldLocation fieldLocation,
      boolean isLeftBranch) {
    this.swerve = swerve;
    this.visionSubsystem = visionSubsystem;
    this.isLeftBranch = isLeftBranch;
    this.fieldLocation = fieldLocation;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();

    // Disable vision processing
    // visionSubsystem.setPoseUpdatesEnabled(false);
    tagId = 0;

    // Setup initial pose and target based on target tag
    initialRobotPose = swerve.getPose();
    computeTarget(initialRobotPose, true);
  }

  @Override
  public void execute() {

    if (noTagsAbort) {
      return;
    }

    // Get current pose, and recalculate our current real world pose based on target tag
    Pose2d currentPose = swerve.getPose();
    computeTarget(currentPose, false);

    double xDif = targetPose.getX() - currentPose.getX();
    double yDif = targetPose.getY() - currentPose.getY();

    double xS = calcSwerveSpeed(xDif, 0.02, 0.2, 1);
    double yS = calcSwerveSpeed(yDif, 0.02, 0.2, 1);

    swerve.driveFieldOriented(xS, yS, swerve.computeOmega(targetHeadingDeg));
  }

  private void computeTarget(Pose2d currentPose, boolean initalize) {

    if (initalize) {
      // If No field location, use the closest tag
      if (fieldLocation == null) {
        int visionClosestTagId = (int) visionSubsystem.getVisibleTargetTagId();
        if (visionClosestTagId < 1 || visionClosestTagId > 22) {
          noTagsAbort = true;
          return;
        }
        tagId = visionClosestTagId;
      } else {
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
          tagId = fieldLocation.redTagId;
        } else {
          tagId = fieldLocation.blueTagId;
        }
        isLeftBranch = fieldLocation.isLeftSide;
      }

      tagPose = Constants.FieldConstants.TAGS.getTagById(tagId).pose;
      this.targetHeadingDeg = tagPose.getRotation().getDegrees();

      // Whatever tag we're looking at, it's the one we want.
      visionSubsystem.setTargetTagId(tagId);
    } else {
      // Need to make sure target tag is in view to do this
      if (!visionSubsystem.isTagInView(tagId)) {
        return;
      }
    }

    double robotHeading = currentPose.getRotation().getDegrees();
    double targetAngleRelative = visionSubsystem.angleToTarget();
    double distanceToTarget = visionSubsystem.distanceTagToRobot();

    // Compute target position relative to robot
    double targetGlobalAngle = robotHeading + targetAngleRelative;
    double targetX = distanceToTarget * Math.cos(Math.toRadians(targetGlobalAngle));
    double targetY = distanceToTarget * Math.sin(Math.toRadians(targetGlobalAngle));

    Pose2d newPose =
        new Pose2d(tagPose.getX() - targetX, tagPose.getY() - targetY, currentPose.getRotation());

    swerve.resetOdometry(newPose);

    if (initalize) {
      double sideOffset = isLeftBranch ? OFFSET_FROM_TAG_FOR_SCORING : -OFFSET_FROM_TAG_FOR_SCORING;

      // Compute side offset along target's orientation (Perpendicular)
      double sideOffsetX = sideOffset * Math.cos(Math.toRadians(targetHeadingDeg + 90));
      double sideOffsetY = sideOffset * Math.sin(Math.toRadians(targetHeadingDeg + 90));

      // Compute backward offset away from the target (Opposite direction)
      double backwardOffsetX =
          OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.cos(Math.toRadians(targetHeadingDeg + 180));
      double backwardOffsetY =
          OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.sin(Math.toRadians(targetHeadingDeg + 180));

      // Compute final target position with offset
      double finalX = targetX + sideOffsetX + backwardOffsetX;
      double finalY = targetY + sideOffsetY + backwardOffsetY;

      targetPose =
          new Pose2d(
              newPose.getX() + finalX,
              newPose.getY() + finalY,
              Rotation2d.fromDegrees(targetHeadingDeg));

      SmartDashboard.putNumber("1310/DriveToScorePositionCommand/finalX", finalX);
      SmartDashboard.putNumber("1310/DriveToScorePositionCommand/finalY", finalY);
      SmartDashboard.putNumber("1310/DriveToScorePositionCommand/targetIagId", tagId);
    }

    SmartDashboard.putNumber("1310/DriveToScorePositionCommand/targetX", targetX);
    SmartDashboard.putNumber("1310/DriveToScorePositionCommand/targetY", targetY);
    SmartDashboard.putNumber("1310/DriveToScorePositionCommand/targetAngle", targetGlobalAngle);
    SmartDashboard.putNumber("1310/DriveToScorePositionCommand/targetTagId", tagId);
    SmartDashboard.putString(
        "1310/DriveToScorePositionCommand/initalPose", initialRobotPose.toString());
    SmartDashboard.putString("1310/DriveToScorePositionCommand/myPose", newPose.toString());
    SmartDashboard.putString("1310/DriveToScorePositionCommand/targetPose", targetPose.toString());
  }

  private double calcSwerveSpeed(
      double distance, double tolerance, double minSpeed, double maxSpeed) {
    double xSpeed = Math.max(minSpeed, Math.min(maxSpeed, Math.abs(distance)));
    if (Math.abs(distance) < tolerance) {
      xSpeed = 0;
    }
    return xSpeed * Math.signum(distance);
  }

  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    visionSubsystem.setTargetTagId(0);
    swerve.stop();
  }

  @Override
  public boolean isFinished() {
    if (noTagsAbort) {
      return true;
    }

    boolean done =
        (SwerveUtils.isCloseEnough(
                swerve.getPose().getTranslation(), targetPose.getTranslation(), 0.02)
            && SwerveUtils.isCloseEnough(swerve.getYaw(), targetHeadingDeg, 2));

    if (done) {
      Pose2d endPose = swerve.getPose();
      SmartDashboard.putString(
          "1310/DriveToScorePositionCommand/endPose",
          "x["
              + endPose.getX()
              + "] y["
              + endPose.getY()
              + "], deg["
              + endPose.getRotation().getDegrees()
              + "]");
      visionSubsystem.setTargetTagId(0);
    }
    return done;
  }
}
