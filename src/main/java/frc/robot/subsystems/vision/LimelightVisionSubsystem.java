package frc.robot.subsystems.vision;

import ca.team1310.swerve.vision.VisionPoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.telemetry.Telemetry;

public class LimelightVisionSubsystem extends SubsystemBase {

  // Standard Deviations Used for most of the Pose Updates
  private static final Matrix<N3, N1> MEGATAG1_STDDEV = VecBuilder.fill(0.01, 0.01, 0.05);
  private static final Matrix<N3, N1> MEGATAG2_STDDEV = VecBuilder.fill(0.06, 0.06, 9999999);

  // Orientation publishers
  private final DoubleArrayPublisher nikolaRobotOrientation;
  private final DoubleArrayPublisher thomasRobotOrientation;

  // MegaTags
  private final DoubleArraySubscriber nikolaMegaTag;
  private final DoubleArraySubscriber thomasMegaTag;

  // These hold the data from the limelights, updated every periodic()
  private final LimelightBotPose nikolaBotPoseCache = new LimelightBotPose();
  private final LimelightBotPose thomasBotPoseCache = new LimelightBotPose();

  private final SwerveSubsystem swerve;
  private final double maxAmbiguity;
  private final double highQualityAmbiguity;
  private final double maxVisposDeltaDistanceMetres;
  private final boolean megatag2;

  private boolean poseUpdatesEnabled = true;

  public LimelightVisionSubsystem(VisionConfig visionConfig, SwerveSubsystem swerve) {
    this.maxAmbiguity = visionConfig.maxAmbiguity();
    this.highQualityAmbiguity = visionConfig.highQualityAmbiguity();
    this.maxVisposDeltaDistanceMetres = visionConfig.maxVisposeDeltaDistanceMetres();
    this.megatag2 = visionConfig.megatag2();
    Telemetry.vision.telemetryLevel = visionConfig.telemetryLevel();
    this.swerve = swerve;

    final NetworkTable nikola = NetworkTableInstance.getDefault().getTable("limelight-nikola");
    final NetworkTable thomas = NetworkTableInstance.getDefault().getTable("limelight-thomas");

    nikolaRobotOrientation = nikola.getDoubleArrayTopic("robot_orientation_set").publish();
    thomasRobotOrientation = thomas.getDoubleArrayTopic("robot_orientation_set").publish();

    // Initialize the NT subscribers for whichever of MT1/2 is used
    if (megatag2) {
      nikolaMegaTag = nikola.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
      thomasMegaTag = thomas.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    } else {
      nikolaMegaTag = nikola.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
      thomasMegaTag = thomas.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
    }

    // inputs/configs
    nikola.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    nikola.getEntry("camMode").setNumber(visionConfig.camModeVision());
    thomas.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    thomas.getEntry("camMode").setNumber(visionConfig.camModeVision());
  }

  @Override
  public void periodic() {
    // Update the limelight and let it know our orientation for MegaTag2
    double[] orientationSet = new double[] {swerve.getYaw(), 0, 0, 0, 0, 0};
    nikolaRobotOrientation.set(orientationSet);
    thomasRobotOrientation.set(orientationSet);

    // Pull data from the limelights and update our cache
    nikolaBotPoseCache.update(nikolaMegaTag.getAtomic());
    thomasBotPoseCache.update(thomasMegaTag.getAtomic());

    // Publish the pose estimate to the PoseEstimator, and update telemetry
    publishVisionPoseMeasurement();

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
   * Enable or disable pose updates from the vision system
   *
   * @param enabled true to enable, false to disable
   */
  public void setPoseUpdatesEnabled(boolean enabled) {
    poseUpdatesEnabled = enabled;
  }

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

  /**
   * Get the pose estimate from the vision system and update swerve's odometry. This function will
   * check there is a valid pose to be used, and then obtain either the MegaTag2 or MegaTag1 pose
   * based on config.
   */
  private void publishVisionPoseMeasurement() {
    // Make sure tags are visible and pose is on-field
    VisionPoseEstimate visionPoseEstimate = null;

    if (poseUpdatesEnabled && nikolaBotPoseCache.isPoseValid()) {
      if (megatag2) {
        visionPoseEstimate = getVisionPoseMeasurementMegaTag2();
      } else {
        visionPoseEstimate = getVisionPoseMeasurementMegaTag1();
      }
    }

    // If we have an update, send it to swerve.  If not, set PoseConfidence to NONE
    if (visionPoseEstimate != null) {
      swerve.addVisionMeasurement(visionPoseEstimate);
    } else {
      Telemetry.vision.poseConfidence = LimelightPoseEstimate.PoseConfidence.NONE;
    }
  }

  /**
   * Get the vision pose measurement for MegaTag2. If it exists, it's pretty high quality, so use
   * it.
   *
   * @return The vision pose measurement for MegaTag2
   */
  private VisionPoseEstimate getVisionPoseMeasurementMegaTag2() {
    Telemetry.vision.poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG2;
    return new LimelightPoseEstimate(
        nikolaBotPoseCache.getPose(),
        nikolaBotPoseCache.getTimestampSeconds() - nikolaBotPoseCache.getTotalLatencySeconds(),
        MEGATAG2_STDDEV);
  }

  /**
   * Get the vision pose measurement for MegaTag1. Logic is as follows:
   *
   * <ul>
   *   <li>Check tag ambiguity - if it's very low (<0.1) it means we've got a high confidence set of
   *       data, so use low standard deviation.
   *   <li>If ambiguity is medium (0.1-0.7), use distance from tag to scale standard deviations and
   *       trust this data as medium confidence.
   *   <li>If ambiguity is > 0.7, don't use the data at all.
   * </ul>
   *
   * @return The vision pose measurement for MegaTag1, or null if no acceptable data
   */
  private VisionPoseEstimate getVisionPoseMeasurementMegaTag1() {
    double tagAmbiguity = nikolaBotPoseCache.getTagAmbiguity(0);

    if (tagAmbiguity < maxAmbiguity) {

      if (tagAmbiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
        // If the ambiguity is very low (< 0.1) or DriverStation disabled, high confidence pose
        // update
        Telemetry.vision.poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1_HIGH;
        return new LimelightPoseEstimate(
            nikolaBotPoseCache.getPose(),
            nikolaBotPoseCache.getTimestampSeconds(),
            MEGATAG1_STDDEV);
      } else {
        // For medium Ambiguity, only use it if we're within 0.5m.  Scale standard deviation by
        // distance.
        double compareDistance =
            nikolaBotPoseCache
                .getPose()
                .getTranslation()
                .getDistance(swerve.getPose().getTranslation());
        if (compareDistance < maxVisposDeltaDistanceMetres) {
          Telemetry.vision.poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1_MED;
          double stdDevRatio = Math.pow(nikolaBotPoseCache.getTagDistToRobot(0), 2) / 2;
          Matrix<N3, N1> deviations = VecBuilder.fill(stdDevRatio, stdDevRatio, stdDevRatio * 5);
          return new LimelightPoseEstimate(
              nikolaBotPoseCache.getPose(), nikolaBotPoseCache.getTimestampSeconds(), deviations);
        }
      }
    }

    // If we make it here, we're lacking good vision data
    return null;
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
