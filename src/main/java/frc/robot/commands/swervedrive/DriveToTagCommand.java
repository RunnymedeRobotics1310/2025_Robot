package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/**
 * Drive to a scoring position based on a vision target tag. If no tag is specified, it'll use the
 * currently visible closest one. This will calculate a distance that needs to be travelled, and
 * will tolerate losing sight of the tag for a short duration. It will stop moving once it reaches
 * the target.
 */
public class DriveToTagCommand extends LoggingCommand {

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
  private boolean noDataTimeout = false;
  private boolean reachedTarget = false;

  private double offsetX;
  private double offsetY;

  private double speedX;
  private double speedY;
  private double omega;

  private double lastUpdateTime = 0;
  private double travelTimeEstimate = 0;
  private double timeRemaining = 0;

  public static final double OFFSET_FROM_TAG_FOR_SCORING = 0.20;
  public static final double OFFSET_FROM_TAG_ROBOT_HALF_LENGTH = 0.57;

  private static final double DATA_TIMEOUT = 0.2;
  private static final double MAX_SPEED = 1.5; // Maximum speed in m/s
  private static final double SLOWDOWN_DISTANCE =
      0.3; // Distance at which to start slowing down (30 cm)
  private static final double STOP_TOLERANCE = 0.2;

  /**
   * Drive to a scoring position based on a vision target tag. If no tag is specified, it'll use the
   * currently visible closest one
   *
   * @param swerve The swerve subsystem
   * @param visionSubsystem The vision subsystem
   * @param fieldLocation The field location to drive to
   * @param isLeftBranch Should this target the left branch or right?
   */
  public DriveToTagCommand(
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
    speedX = 0;
    speedY = 0;
    omega = 0;
    lastUpdateTime = 0;
    travelTimeEstimate = 0;
    timeRemaining = 0;
    noTagsAbort = false;
    noDataTimeout = false;
    reachedTarget = false;

    // Setup initial pose and target based on target tag
    initialRobotPose = swerve.getPose();

    // If No field location, use the closest tag
    if (fieldLocation == null) {
      int visionClosestTagId = (int) visionSubsystem.getVisibleTargetTagId();
      if (!Constants.FieldConstants.TAGS.isValidTagId(visionClosestTagId)) {
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

    double sideOffset = isLeftBranch ? OFFSET_FROM_TAG_FOR_SCORING : -OFFSET_FROM_TAG_FOR_SCORING;

    // Compute side offset along target's orientation (Perpendicular)
    double sideOffsetX = sideOffset * Math.cos(Math.toRadians(targetHeadingDeg + 90));
    double sideOffsetY = sideOffset * Math.sin(Math.toRadians(targetHeadingDeg + 90));

    // Compute backward offset away from the target (Opposite direction)
    double backwardOffsetX =
        OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.cos(Math.toRadians(targetHeadingDeg + 180));
    double backwardOffsetY =
        OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.sin(Math.toRadians(targetHeadingDeg + 180));

    offsetX = sideOffsetX + backwardOffsetX;
    offsetY = sideOffsetY + backwardOffsetY;

    lastUpdateTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("1310/DriveToTagCommand/targetIagId", tagId);
    SmartDashboard.putString("1310/DriveToTagCommand/initalPose", initialRobotPose.toString());
  }

  @Override
  public void execute() {

    double currentTime = Timer.getFPGATimestamp();
    boolean hasNewData = visionSubsystem.isTagInView(tagId);

    // Update targeting and time remaining when the tag is in view.  This means that if the tag
    // isn't in view, the robot will keep moving towards it in the direction it was last told to go,
    // for the estimated duration.
    if (hasNewData) {
      double robotHeading = swerve.getYaw();
      double targetAngleRelative = visionSubsystem.angleToTarget();
      double distanceToTarget = visionSubsystem.distanceToTarget();
      lastUpdateTime = Timer.getFPGATimestamp();

      // Compute target position relative to robot
      double targetGlobalAngle = robotHeading + targetAngleRelative;
      double targetX = distanceToTarget * Math.cos(Math.toRadians(targetGlobalAngle));
      double targetY = distanceToTarget * Math.sin(Math.toRadians(targetGlobalAngle));

      // Compute final target position with offset
      double finalX = targetX + offsetX;
      double finalY = targetY + offsetY;

      calculateSpeeds(finalX, finalY, robotHeading, targetGlobalAngle);
      swerve.driveRobotOriented(speedX, speedY, omega);

      SmartDashboard.putNumber("1310/DriveToTagCommand/finalX", finalX);
      SmartDashboard.putNumber("1310/DriveToTagCommand/finalY", finalY);
      SmartDashboard.putNumber("1310/DriveToTagCommand/targetX", targetX);
      SmartDashboard.putNumber("1310/DriveToTagCommand/targetY", targetY);
      SmartDashboard.putNumber("1310/DriveToTagCommand/targetAngle", targetGlobalAngle);
    } else {
      // If no new data for a while, we have to abort/stop.
      if (currentTime - lastUpdateTime > DATA_TIMEOUT) {
        noDataTimeout = true;
        swerve.stop();
      }

      // Reduce the remaining time by the amount of time elasped since the last data update
      timeRemaining = travelTimeEstimate - (currentTime - lastUpdateTime);
    }
  }

  private void calculateSpeeds(
      double distanceX, double distanceY, double headingAngle, double targetAngleRobotRelative) {
    double distanceTotal = Math.hypot(distanceX, distanceY); // Total distance to target
    double speedMultiplier = 1.0;

    // Stop if within tolerance, or slow down if within the slowdown distance
    if (distanceTotal < STOP_TOLERANCE) {
      speedMultiplier = 0;
      reachedTarget = true;
    } else if (distanceTotal < SLOWDOWN_DISTANCE) {
      speedMultiplier = distanceTotal / SLOWDOWN_DISTANCE; // Scale speed linearly
    }

    // Normalize direction and apply speed
    speedX = speedMultiplier * MAX_SPEED * Math.cos(targetAngleRobotRelative);
    speedY = speedMultiplier * MAX_SPEED * Math.sin(targetAngleRobotRelative);
    omega = swerve.computeOmega(headingAngle);

    // Time estimate remaining
    timeRemaining =
        travelTimeEstimate =
            (distanceTotal >= STOP_TOLERANCE) ? (distanceTotal / (speedMultiplier * MAX_SPEED)) : 0;
  }

  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    visionSubsystem.setTargetTagId(0);
    swerve.stop();
  }

  @Override
  public boolean isFinished() {
    if (noTagsAbort) {
      log("No tags to target");
      return true;
    }

    if (noDataTimeout) {
      log("No data for too long, stopping");
      return true;
    }

    if (reachedTarget) {
      log("Reached target");
      Pose2d endPose = swerve.getPose();
      SmartDashboard.putString(
          "1310/DriveToTagCommand/endPose",
          "x["
              + endPose.getX()
              + "] y["
              + endPose.getY()
              + "], deg["
              + endPose.getRotation().getDegrees()
              + "]");
      return true;
    }

    return false;
  }
}
