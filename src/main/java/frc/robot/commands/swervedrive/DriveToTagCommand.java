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

  //  public static final double OFFSET_FROM_TAG_FOR_SCORING = 0.20;
  public static final double OFFSET_FROM_TAG_ROBOT_HALF_LENGTH = 0.57;

  private static final double DATA_TIMEOUT = 0.2;
  private static final double MAX_SPEED = 1.5; // Maximum speed in m/s
  private static final double SLOWDOWN_DISTANCE =
      0.3; // Distance at which to start slowing down (30 cm)
  private static final double STOP_TOLERANCE = 0.02;

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

    // Reset basic variables
    tagId = 0;
    speedX = speedY = omega = lastUpdateTime = travelTimeEstimate = 0;
    noTagsAbort = noDataTimeout = reachedTarget = false;

    // If No field location, use the closest tag
    Pose2d tagPose = initTag();
    if (tagPose == null) {
      noTagsAbort = true;
      return;
    }

    // Whatever tag we're looking at, it's the one we want.
    visionSubsystem.setTargetTagId(tagId);

    // Calculate the target position based on tag with side and backwards offsets
    calcTargetPosition(tagPose.getRotation().getDegrees());

    // Initialize our data
    calcTarget();

    SmartDashboard.putNumber("1310/DriveToTagCommand/targetIagId", tagId);
    SmartDashboard.putString("1310/DriveToTagCommand/initalPose", swerve.getPose().toString());
  }

  /**
   * Initialize the tag to target. If a field location is specified, it will use the tag id from the
   * fieldLocation, otherwise will use whatever tag is visible
   *
   * @return Tag Pose for selected Tag
   */
  private Pose2d initTag() {
    if (fieldLocation == null) {
      int visionClosestTagId = (int) visionSubsystem.getVisibleTargetTagId(isLeftBranch);
      if (!Constants.FieldConstants.TAGS.isValidTagId(visionClosestTagId)) {
        return null;
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

    return Constants.FieldConstants.TAGS.getTagById(tagId).pose;
  }

  /**
   * Calculate the target position based on sideways and backwards offsets to the tag location to
   * accommodate for reef positions and size of robot
   *
   * @param targetHeadingDeg The field orientation facing the tag
   */
  private void calcTargetPosition(double targetHeadingDeg) {
    //    double sideOffset = isLeftBranch ? OFFSET_FROM_TAG_FOR_SCORING :
    // -OFFSET_FROM_TAG_FOR_SCORING;
    //
    //    // Compute side offset along target's orientation (Perpendicular)
    //    double sideOffsetX = sideOffset * Math.cos(Math.toRadians(targetHeadingDeg + 90));
    //    double sideOffsetY = sideOffset * Math.sin(Math.toRadians(targetHeadingDeg + 90));

    // Compute backward offset away from the target (Opposite direction)
    double backwardOffsetX =
        OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.cos(Math.toRadians(targetHeadingDeg + 180));
    double backwardOffsetY =
        OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.sin(Math.toRadians(targetHeadingDeg + 180));

    offsetX = /*sideOffsetX +*/ backwardOffsetX;
    offsetY = /*sideOffsetY +*/ backwardOffsetY;
  }

  private boolean calcTarget() {
    boolean tagInView = visionSubsystem.isTagInView(tagId);

    if (tagInView) {
      // Get Target info from Limelight & Swerve
      double robotHeading = swerve.getYaw();
      double targetAngleRelative = visionSubsystem.angleToTarget(isLeftBranch);
      double distanceToTarget = visionSubsystem.distanceTagToRobot(isLeftBranch);
      lastUpdateTime = Timer.getFPGATimestamp();

      // Compute target position relative to robot
      double targetGlobalAngle = robotHeading + targetAngleRelative;
      double targetX = distanceToTarget * Math.cos(Math.toRadians(targetGlobalAngle));
      double targetY = distanceToTarget * Math.sin(Math.toRadians(targetGlobalAngle));

      // Compute final target position with targeting offset calculated during init
      double distanceX = targetX + offsetX;
      double distanceY = targetY + offsetY;

      // Determine Motor Speeds & Omega
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
      speedX = speedMultiplier * MAX_SPEED * Math.cos(targetAngleRelative);
      speedY = speedMultiplier * MAX_SPEED * Math.sin(targetAngleRelative);
      omega = swerve.computeOmega(targetGlobalAngle);

      // Time estimate remaining
      travelTimeEstimate =
          (distanceTotal >= STOP_TOLERANCE) ? (distanceTotal / (speedMultiplier * MAX_SPEED)) : 0;

      SmartDashboard.putNumber("1310/DriveToTagCommand/distanceX", distanceX);
      SmartDashboard.putNumber("1310/DriveToTagCommand/distanceY", distanceY);
      SmartDashboard.putNumber("1310/DriveToTagCommand/targetX", targetX);
      SmartDashboard.putNumber("1310/DriveToTagCommand/targetY", targetY);
      SmartDashboard.putNumber("1310/DriveToTagCommand/targetAngle", targetGlobalAngle);
    }

    return tagInView;
  }

  @Override
  public void execute() {

    double currentTime = Timer.getFPGATimestamp();
    boolean hasNewData = calcTarget();

    // Update targeting and time remaining when the tag is in view.  This means that if the tag
    // isn't in view, the robot will keep moving towards it in the direction it was last told to go,
    // for the estimated duration.
    if (hasNewData) {
      swerve.driveRobotOriented(speedX, speedY, omega);
    } else {
      // If no new data for a while, we have to abort/stop.
      double timeSinceLastData = currentTime - lastUpdateTime;
      if (timeSinceLastData > DATA_TIMEOUT) {
        noDataTimeout = true;
        swerve.stop();
      }

      // Reduce the remaining time by the amount of time elapsed since the last data update
      double timeRemaining = travelTimeEstimate - timeSinceLastData;

      // If we're out of time, stop moving, we should have reached the target
      // The travelTimeEstimate > 0 handles the case where a tag wasn't visible on the 1st calls to
      // execute() within DATA_TIMEOUT window - i.e. we can wait a little to see if the tag shows up
      if (travelTimeEstimate > 0 && timeRemaining <= 0) {
        reachedTarget = true;
        swerve.stop();
      }
    }
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
