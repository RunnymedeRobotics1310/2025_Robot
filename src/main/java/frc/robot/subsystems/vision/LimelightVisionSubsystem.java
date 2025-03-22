package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;

public class LimelightVisionSubsystem extends SubsystemBase {

  // MegaTags
  private final DoubleArraySubscriber nikolaMegaTag;
  private final DoubleArraySubscriber thomasMegaTag;

  // These hold the data from the limelights, updated every periodic()
  private final LimelightBotPose nikolaBotPoseCache = new LimelightBotPose();
  private final LimelightBotPose thomasBotPoseCache = new LimelightBotPose();

  private final SwerveSubsystem swerve;

  public LimelightVisionSubsystem(VisionConfig visionConfig, SwerveSubsystem swerve) {
    this.swerve = swerve;

    final NetworkTable nikola = NetworkTableInstance.getDefault().getTable("limelight-nikola");
    final NetworkTable thomas = NetworkTableInstance.getDefault().getTable("limelight-thomas");

    // Initialize the NT subscribers for whichever of MT1/2 is used
    nikolaMegaTag = nikola.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    thomasMegaTag = thomas.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

    // inputs/configs
    nikola.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    nikola.getEntry("camMode").setNumber(visionConfig.camModeVision());
    thomas.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    thomas.getEntry("camMode").setNumber(visionConfig.camModeVision());
  }

  @Override
  public void periodic() {
    // Pull data from the limelights and update our cache
    nikolaBotPoseCache.update(nikolaMegaTag.getAtomic());
    thomasBotPoseCache.update(thomasMegaTag.getAtomic());

    // Update telemetry
    updateTelemetry();
  }

  /**
   * If targeting the left reef, use the left reef pose, otherwise use the right reef pose
   *
   * @param leftBranch Are we targeting the left branch of a coral, or right branch?
   * @return Appropriate botPose data for Nikola or Thomas based on side
   */
  private LimelightBotPose getBotPose(boolean leftBranch) {
    return leftBranch ? nikolaBotPoseCache : thomasBotPoseCache;
  }

  /* Public API */

  /**
   * Get the tag ID of the closest visible target to default limelight (nikola)
   *
   * @return the tag ID of the closest visible target to default limelight (nikola)
   */
  public double getVisibleTargetTagId() {
    return getVisibleTargetTagId(true);
  }

  /**
   * Get the tag ID of the closest visible target to the limelight handling left or right branch
   *
   * @param leftBranch Left or Right branch?
   * @return the tag ID of the closest visible target to the limelight handling left or right branch
   */
  public double getVisibleTargetTagId(boolean leftBranch) {
    return getBotPose(leftBranch).getTagId(0);
  }

  /**
   * Get the number of tags visible to the default limelight (nikola)
   *
   * @return the number of tags visible to the default limelight (nikola)
   */
  public int getNumTagsVisible() {
    return (int) nikolaBotPoseCache.getTagCount();
  }

  /**
   * Obtain the distance to robot centre to the tag either nearest to, or targeted if one has been
   * set by setTargetTag(), to the default limelight (nikola)
   *
   * @return the distance to robot centre to the nearest or targeted tag
   */
  public double distanceTagToRobot() {
    return distanceTagToRobot(0, true);
  }

  /**
   * Obtain the distance to robot centre to the tag either nearest to, or targeted if one has been
   * set by setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @param leftBranch Left or Right branch?
   * @return the distance to robot centre to the nearest or targeted tag
   */
  public double distanceTagToRobot(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose(leftBranch);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return botPose.getTagDistToRobot(index);
  }

  /**
   * Obtain the distance to the camera for tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the default limelight (nikola)
   *
   * @return the distance to camera to the nearest or targeted tag
   */
  public double distanceTagToCamera() {
    return distanceTagToCamera(0, true);
  }

  /**
   * Obtain the distance to camera to the tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @param leftBranch Left or Right branch?
   * @return the distance to camera to the nearest or targeted tag
   */
  public double distanceTagToCamera(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose(leftBranch);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return botPose.getTagDistToCamera(index);
  }

  /**
   * Obtain the angle to the tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the default limelight (nikola)
   *
   * @return the angle to the nearest or targeted tag
   */
  public double angleToTarget() {
    return angleToTarget(0, true);
  }

  /**
   * Obtain the angle to the tag either nearest to, or targeted if one has been set by
   * setTargetTag(), to the limelight handling left or right branch.
   *
   * @param tagId Tag to use, or 0 if looking for nearest tag
   * @param leftBranch Left or Right branch?
   * @return the angle to the nearest or targeted tag
   */
  public double angleToTarget(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose(leftBranch);

    int index = 0;
    if (tagId > 0) {
      index = botPose.getTagIndex(tagId);
    }
    return -botPose.getTagTxnc(index);
  }

  /**
   * Get the number of tags visible to the default limelight (nikola)
   *
   * @return the number of tags visible to the default limelight (nikola)
   */
  public double getTagCount() {
    return nikolaBotPoseCache.getTagCount();
  }

  /**
   * Checks if a specific tag is visible to the default limelight (nikola)
   *
   * @param tagId The ID of the tag to check
   * @return If tagId is visible or not
   */
  public boolean isTagInView(int tagId) {
    return isTagInView(tagId, true);
  }

  /**
   * Checks if a specific tag is visible to the limelight handling left or right branches.
   *
   * @param tagId The ID of the tag to check
   * @return If tagId is visible or not
   */
  public boolean isTagInView(int tagId, boolean leftBranch) {
    LimelightBotPose botPose = getBotPose(leftBranch);
    return botPose.getTagIndex(tagId) != -1;
  }
  
  /** Update telemetry with vision data */
  private void updateTelemetry() {
    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.REGULAR
        || Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {

      Pose2d odometryPose = swerve.getPose();
      double yaw = swerve.getYaw();

      double compareDistance =
          nikolaBotPoseCache.getPose().getTranslation().getDistance(odometryPose.getTranslation());
      double compareHeading =
          nikolaBotPoseCache.getPose().getRotation().getDegrees()
              - odometryPose.getRotation().getDegrees();

      Telemetry.vision.poseDeltaMetres = compareDistance;
      Telemetry.vision.headingDeltaDegrees = compareHeading;
      Telemetry.vision.poseMetresX = odometryPose.getX();
      Telemetry.vision.poseMetresY = odometryPose.getY();
      Telemetry.vision.poseHeadingDegrees = odometryPose.getRotation().getDegrees();
      Telemetry.vision.visionPoseX = nikolaBotPoseCache.getPoseX();
      Telemetry.vision.visionPoseY = nikolaBotPoseCache.getPoseY();
      Telemetry.vision.visionPoseHeading = nikolaBotPoseCache.getPoseRotationYaw();
      Telemetry.vision.navxYaw = yaw;
      Telemetry.vision.navxYawDelta = odometryPose.getRotation().getDegrees() - yaw;
    }

    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {
      Telemetry.vision.poseXSeries.add(nikolaBotPoseCache.getPoseX());
      Telemetry.vision.poseYSeries.add(nikolaBotPoseCache.getPoseY());
      Telemetry.vision.poseDegSeries.add(nikolaBotPoseCache.getPoseRotationYaw());

      Telemetry.vision.nikVisibleTags = nikolaBotPoseCache.getVisibleTags();
      Telemetry.vision.nikTx = nikolaBotPoseCache.getTagTxnc(0);
      Telemetry.vision.nikDistanceToRobot = nikolaBotPoseCache.getTagDistToRobot(0);
      Telemetry.vision.nikDistanceToCam = nikolaBotPoseCache.getTagDistToCamera(0);

      Telemetry.vision.tomVisibleTags = thomasBotPoseCache.getVisibleTags();
      Telemetry.vision.tomTx = thomasBotPoseCache.getTagTxnc(0);
      Telemetry.vision.tomDistanceToRobot = thomasBotPoseCache.getTagDistToRobot(0);
      Telemetry.vision.tomDistanceToCam = thomasBotPoseCache.getTagDistToCamera(0);
    }
  }

  @Override
  public String toString() {
    return "AC/DeepSea Vision Subsystem";
  }
}
