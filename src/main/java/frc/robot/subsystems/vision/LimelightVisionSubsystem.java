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

  private final NetworkTable nikolaVision =
      NetworkTableInstance.getDefault().getTable("limelight-nikola");
  private final NetworkTable thomasVision =
      NetworkTableInstance.getDefault().getTable("limelight-thomas");

  private final DoubleArrayPublisher nikolaRobotOrientation =
      nikolaVision.getDoubleArrayTopic("robot_orientation_set").publish();
  private final DoubleArrayPublisher thomasRobotOrientation =
      thomasVision.getDoubleArrayTopic("robot_orientation_set").publish();

  // MegaTags
  private final DoubleArraySubscriber nikolaMegaTag;
  private final DoubleArraySubscriber thomasMegaTag;

  // Standard Deviations Used for most of the Pose Updates
  private final Matrix<N3, N1> poseDeviation;

  // These hold the data from the limelights, updated every periodic()
  private final LimelightBotPose nikolaBotPoseCache = new LimelightBotPose(null, 0);
  private final LimelightBotPose thomasBotPoseCache = new LimelightBotPose(null, 0);

  private final SwerveSubsystem swerve;
  private final double fieldExtentMetresX;
  private final double fieldExtentMetresY;
  private final double maxAmbiguity;
  private final double highQualityAmbiguity;
  private final double maxVisposDeltaDistanceMetres;
  private final boolean megatag2;

  private boolean poseUpdatesEnabled = true;

  public LimelightVisionSubsystem(VisionConfig visionConfig, SwerveSubsystem swerve) {
    this.fieldExtentMetresX = visionConfig.fieldExtentMetresX();
    this.fieldExtentMetresY = visionConfig.fieldExtentMetresY();
    this.maxAmbiguity = visionConfig.maxAmbiguity();
    this.highQualityAmbiguity = visionConfig.highQualityAmbiguity();
    this.maxVisposDeltaDistanceMetres = visionConfig.maxVisposeDeltaDistanceMetres();
    this.megatag2 = visionConfig.megatag2();
    Telemetry.vision.telemetryLevel = visionConfig.telemetryLevel();
    this.swerve = swerve;

    // Initialize the NT subscribers for whichever of MT1/2 is used
    if (megatag2) {
      nikolaMegaTag =
          nikolaVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
      thomasMegaTag =
          thomasVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
      poseDeviation = VecBuilder.fill(0.06, 0.06, 9999999);
    } else {
      nikolaMegaTag = nikolaVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
      thomasMegaTag = thomasVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
      poseDeviation = VecBuilder.fill(0.01, 0.01, 0.05);
    }

    // inputs/configs
    nikolaVision.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    nikolaVision.getEntry("camMode").setNumber(visionConfig.camModeVision());
    thomasVision.getEntry("pipeline").setNumber(visionConfig.pipelineAprilTagDetect());
    thomasVision.getEntry("camMode").setNumber(visionConfig.camModeVision());
  }

  @Override
  public void periodic() {
    // First, update the limelight and let it know our orientation for MegaTag2
    double[] orientationSet = new double[] {swerve.getYaw(), 0, 0, 0, 0, 0};
    nikolaRobotOrientation.set(orientationSet);
    thomasRobotOrientation.set(orientationSet);

    // Next, pull updated data from the limelights
    nikolaBotPoseCache.update(nikolaMegaTag.getAtomic());
    thomasBotPoseCache.update(thomasMegaTag.getAtomic());

    // Lastly, publish the pose estimate to the PoseEstimator
    updateVisionPose();
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
   * Get the pose estimate from the vision system to swerve's odometry
   *
   * <p>Logic In This Function:
   *
   * <ol>
   *   <li>Set the orientation of the robot from odometry into the limelight for MegaTag2 to work
   *   <li>Ensure the latest vision pose data is valid (on field, >=1 tags visible, robot not
   *       spinning like crazy
   *   <li>If megatag2 was set, just use the data with recommended standard deviation data.
   *   <li>If using megatag1:
   *       <ul>
   *         <li>Check tag ambiguity - if it's very low (<0.1) it means we've got a high confidence
   *             set of data, so use low standard deviation.
   *         <li>If ambiguity is medium (0.1-0.7), use distance from tag to scale standard
   *             deviations and trust this data as medium confidence.
   *         <li>If ambiguity is > 0.7, don't use the data at all.
   *       </ul>
   * </ol>
   */
  private void updateVisionPose() {

    VisionPoseEstimate visionPoseEstimate = null;

    // Get the current pose delta
    double compareDistance = -1;
    double compareHeading = -1;
    double tagAmbiguity = nikolaBotPoseCache.getTagAmbiguity(0);
    Pose2d odometryPose = swerve.getPose();
    double yaw = swerve.getYaw();

    LimelightPoseEstimate.PoseConfidence poseConfidence = LimelightPoseEstimate.PoseConfidence.NONE;
    LimelightBotPose botPose = nikolaBotPoseCache;

    // If pose is 0,0 or no tags in view, we don't actually have data - return null
    if (nikolaBotPoseCache.getTagCount() > 0
        && nikolaBotPoseCache.isPoseXInBounds(0, fieldExtentMetresX)
        && nikolaBotPoseCache.isPoseYInBounds(0, fieldExtentMetresY)) {

      if (megatag2) {
        poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG2;
        visionPoseEstimate =
            new LimelightPoseEstimate(
                nikolaBotPoseCache.getPose(),
                nikolaBotPoseCache.getTimestampSeconds()
                    - nikolaBotPoseCache.getTotalLatencySeconds(),
                poseDeviation);
      }
      // MT1: Do we have a decent signal?  i.e. Ambiguity < 0.7
      else if (tagAmbiguity < maxAmbiguity) {
        // If the ambiguity is very low, use the data as is (or when disabled, to allow for bot
        // repositioning
        if (tagAmbiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
          poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1_HIGH;
          visionPoseEstimate =
              new LimelightPoseEstimate(
                  nikolaBotPoseCache.getPose(),
                  nikolaBotPoseCache.getTimestampSeconds(),
                  poseDeviation);
        } else {
          // We need to be careful with this data set. If the location is too far off,
          // don't use it. Otherwise, scale confidence by distance.
          compareDistance =
              botPose.getPose().getTranslation().getDistance(odometryPose.getTranslation());
          if (compareDistance < maxVisposDeltaDistanceMetres) {
            poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1_MED;
            double stdDevRatio = Math.pow(nikolaBotPoseCache.getTagDistToRobot(0), 2) / 2;
            Matrix<N3, N1> deviations = VecBuilder.fill(stdDevRatio, stdDevRatio, stdDevRatio * 5);
            visionPoseEstimate =
                new LimelightPoseEstimate(
                    nikolaBotPoseCache.getPose(),
                    nikolaBotPoseCache.getTimestampSeconds(),
                    deviations);
          }
        }
      }
    }

    // Telemetry Handling
    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.REGULAR
        || Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {

      compareDistance =
          botPose.getPose().getTranslation().getDistance(odometryPose.getTranslation());
      compareHeading =
          botPose.getPose().getRotation().getDegrees() - odometryPose.getRotation().getDegrees();

      Telemetry.vision.tagAmbiguity = tagAmbiguity;
      Telemetry.vision.poseConfidence = poseConfidence;
      Telemetry.vision.poseDeltaMetres = compareDistance;
      Telemetry.vision.headingDeltaDegrees = compareHeading;
      Telemetry.vision.poseMetresX = odometryPose.getX();
      Telemetry.vision.poseMetresY = odometryPose.getY();
      Telemetry.vision.poseHeadingDegrees = odometryPose.getRotation().getDegrees();
      Telemetry.vision.visionPoseX = botPose.getPoseX();
      Telemetry.vision.visionPoseY = botPose.getPoseY();
      Telemetry.vision.visionPoseHeading = botPose.getPoseRotationYaw();
      Telemetry.vision.navxYaw = yaw;
      Telemetry.vision.navxYawDelta = odometryPose.getRotation().getDegrees() - yaw;
    }

    if (Telemetry.vision.telemetryLevel == VisionTelemetryLevel.VERBOSE) {
      Telemetry.vision.poseXSeries.add(botPose.getPoseX());
      Telemetry.vision.poseYSeries.add(botPose.getPoseY());
      Telemetry.vision.poseDegSeries.add(botPose.getPoseRotationYaw());

      Telemetry.vision.nikVisibleTags = nikolaBotPoseCache.getVisibleTags();
      Telemetry.vision.nikTx = nikolaBotPoseCache.getTagTxnc(0);
      Telemetry.vision.nikDistanceToRobot = nikolaBotPoseCache.getTagDistToRobot(0);
      Telemetry.vision.nikDistanceToCam = nikolaBotPoseCache.getTagDistToCamera(0);

      Telemetry.vision.tomVisibleTags = thomasBotPoseCache.getVisibleTags();
      Telemetry.vision.tomTx = thomasBotPoseCache.getTagTxnc(0);
      Telemetry.vision.tomDistanceToRobot = thomasBotPoseCache.getTagDistToRobot(0);
      Telemetry.vision.tomDistanceToCam = thomasBotPoseCache.getTagDistToCamera(0);
    }

    if (poseUpdatesEnabled && visionPoseEstimate != null) {
      swerve.addVisionEstimate(visionPoseEstimate);
    }
  }

  @Override
  public String toString() {
    return "ACDC Vision Subsystem";
  }
}
