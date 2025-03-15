package frc.robot.subsystems.vision;

import ca.team1310.swerve.vision.PoseEstimate;
import ca.team1310.swerve.vision.VisionPoseCallback;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.Telemetry;

public class LimelightVisionSubsystem extends SubsystemBase implements VisionPoseCallback {

  private final NetworkTable nikolaVision =
      NetworkTableInstance.getDefault().getTable("limelight-nikola");
  private final NetworkTable thomasVision =
      NetworkTableInstance.getDefault().getTable("limelight-thomas");

  // inputs/configs
  private final NetworkTableEntry nikolaCamMode = nikolaVision.getEntry("camMode");
  private final NetworkTableEntry nikolaPipeline = nikolaVision.getEntry("pipeline");
  private final DoubleArrayPublisher nikolaRobotOrientation =
      nikolaVision.getDoubleArrayTopic("robot_orientation_set").publish();

  private final NetworkTableEntry thomasCamMode = thomasVision.getEntry("camMode");
  private final NetworkTableEntry thomsPipeline = thomasVision.getEntry("pipeline");
  private final DoubleArrayPublisher thomasRobotOrientation =
      thomasVision.getDoubleArrayTopic("robot_orientation_set").publish();

  // MegaTags
  private final DoubleArraySubscriber nikolaMegaTag1 =
      nikolaVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  private final DoubleArraySubscriber thomasMegaTag1 =
      thomasVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);

  private final DoubleArraySubscriber nikolaMegaTag2 =
      nikolaVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
  private final DoubleArraySubscriber thomasMegaTag2 =
      thomasVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

  // Standard Deviations Used for most of the Pose Updates
  private static final Matrix<N3, N1> POSE_DEVIATION_MEGATAG1 = VecBuilder.fill(0.01, 0.01, 0.05);
  private static final Matrix<N3, N1> POSE_DEVIATION_MEGATAG2 =
      VecBuilder.fill(0.06, 0.06, 9999999);

  // These hold the data from the limelights, updated every periodic()
  private final LimelightBotPose nikolaBotPose = new LimelightBotPose(null, 0);
  private final LimelightBotPose thomasBotPose = new LimelightBotPose(null, 0);

  private final double fieldExtentMetresX;
  private final double fieldExtentMetresY;
  private final double maxAmbiguity;
  private final double highQualityAmbiguity;
  private final double maxVisposDeltaDistanceMetres;
  private final boolean megatag2;

  private boolean poseUpdatesEnabled = true;

  public LimelightVisionSubsystem(VisionConfig visionConfig) {
    this.fieldExtentMetresX = visionConfig.fieldExtentMetresX();
    this.fieldExtentMetresY = visionConfig.fieldExtentMetresY();
    this.maxAmbiguity = visionConfig.maxAmbiguity();
    this.highQualityAmbiguity = visionConfig.highQualityAmbiguity();
    this.maxVisposDeltaDistanceMetres = visionConfig.maxVisposeDeltaDistanceMetres();
    this.megatag2 = visionConfig.megatag2();
    Telemetry.vision.enabled = visionConfig.telemetryEnabled();

    nikolaPipeline.setNumber(visionConfig.pipelineAprilTagDetect());
    nikolaCamMode.setNumber(visionConfig.camModeVision());
    thomsPipeline.setNumber(visionConfig.pipelineAprilTagDetect());
    thomasCamMode.setNumber(visionConfig.camModeVision());
  }

  @Override
  public void periodic() {
    TimestampedDoubleArray nikolaBotPoseBlue = getNikolaMegaTagData();
    nikolaBotPose.update(nikolaBotPoseBlue.value, nikolaBotPoseBlue.timestamp);

    TimestampedDoubleArray thomasBotPoseBlue = getThomasMegaTagData();
    thomasBotPose.update(thomasBotPoseBlue.value, thomasBotPoseBlue.timestamp);
  }

  /**
   * If targeting the left reef, use the left reef pose, otherwise use the right reef pose
   *
   * @param leftBranch Are we targeting the left branch of a coral, or right branch?
   * @return Appropriate botPose data for Nikola or Thomas based on side
   */
  private LimelightBotPose getBotPose(boolean leftBranch) {
    return leftBranch ? nikolaBotPose : thomasBotPose;
  }

  /**
   * Get the latest MegaTag 1 or 2 data from Nikola limelight
   *
   * @return the latest MegaTag data
   */
  private TimestampedDoubleArray getNikolaMegaTagData() {
    return megatag2 ? nikolaMegaTag2.getAtomic() : nikolaMegaTag1.getAtomic();
  }

  /**
   * Get the latest MegaTag 1 or 2 data from Thomas limelight
   *
   * @return the latest MegaTag data
   */
  private TimestampedDoubleArray getThomasMegaTagData() {
    return megatag2 ? thomasMegaTag2.getAtomic() : thomasMegaTag1.getAtomic();
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
    return (int) nikolaBotPose.getTagCount();
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
    return nikolaBotPose.getTagCount();
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
   * Get the pose estimate from the vision system, for callback via VisionPoseCallback
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
   *
   * @param odometryPose the current odometry pose of the robot
   * @param yaw the current yaw of the robot
   * @param yawRate the current yaw rate of the robot
   * @return the pose estimate
   */
  public PoseEstimate getPoseEstimate(Pose2d odometryPose, double yaw, double yawRate) {

    // First, update the limelight and let it know our orientation for MegaTag2
    double[] orientationSet = new double[] {yaw, 0, 0, 0, 0, 0};
    nikolaRobotOrientation.set(orientationSet);
    thomasRobotOrientation.set(orientationSet);

    PoseEstimate returnVal = null;

    // Get the current pose delta
    double compareDistance = -1;
    double compareHeading = -1;
    double tagAmbiguity = nikolaBotPose.getTagAmbiguity(0);

    LimelightPoseEstimate.PoseConfidence poseConfidence = LimelightPoseEstimate.PoseConfidence.NONE;
    LimelightBotPose botPose = nikolaBotPose;

    // If pose is 0,0 or no tags in view, we don't actually have data - return null
    if (nikolaBotPose.getTagCount() > 0
        && nikolaBotPose.isPoseXInBounds(0, fieldExtentMetresX)
        && nikolaBotPose.isPoseYInBounds(0, fieldExtentMetresY)) {

      if (megatag2) {
        poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG2;
        returnVal =
            new LimelightPoseEstimate(
                nikolaBotPose.getPose(),
                nikolaBotPose.getTimestampSeconds() - nikolaBotPose.getTotalLatencySeconds(),
                POSE_DEVIATION_MEGATAG2);
      }
      // MT1: Do we have a decent signal?  i.e. Ambiguity < 0.7
      else if (tagAmbiguity < maxAmbiguity) {
        // If the ambiguity is very low, use the data as is (or when disabled, to allow for bot
        // repositioning
        if (tagAmbiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
          poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1_HIGH;
          returnVal =
              new LimelightPoseEstimate(
                  nikolaBotPose.getPose(),
                  nikolaBotPose.getTimestampSeconds(),
                  POSE_DEVIATION_MEGATAG1);
        } else {
          // We need to be careful with this data set. If the location is too far off,
          // don't use it. Otherwise, scale confidence by distance.
          compareDistance =
              botPose.getPose().getTranslation().getDistance(odometryPose.getTranslation());
          if (compareDistance < maxVisposDeltaDistanceMetres) {
            poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1_MED;
            double stdDevRatio = Math.pow(nikolaBotPose.getTagDistToRobot(0), 2) / 2;
            Matrix<N3, N1> deviations = VecBuilder.fill(stdDevRatio, stdDevRatio, stdDevRatio * 5);
            returnVal =
                new LimelightPoseEstimate(
                    nikolaBotPose.getPose(), nikolaBotPose.getTimestampSeconds(), deviations);
          }
        }
      }
    }

    // Telemetry Handling
    if (Telemetry.vision.enabled) {
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
      Telemetry.vision.poseXSeries.add(botPose.getPoseX());
      Telemetry.vision.poseYSeries.add(botPose.getPoseY());
      Telemetry.vision.poseDegSeries.add(botPose.getPoseRotationYaw());

      Telemetry.vision.nikVisibleTags = nikolaBotPose.getVisibleTags();
      Telemetry.vision.nikTx = nikolaBotPose.getTagTxnc(0);
      Telemetry.vision.nikDistanceToRobot = nikolaBotPose.getTagDistToRobot(0);
      Telemetry.vision.nikDistanceToCam = nikolaBotPose.getTagDistToCamera(0);

      Telemetry.vision.tomVisibleTags = thomasBotPose.getVisibleTags();
      Telemetry.vision.tomTx = thomasBotPose.getTagTxnc(0);
      Telemetry.vision.tomDistanceToRobot = thomasBotPose.getTagDistToRobot(0);
      Telemetry.vision.tomDistanceToCam = thomasBotPose.getTagDistToCamera(0);
    }

    return poseUpdatesEnabled ? returnVal : null;
  }

  @Override
  public String toString() {
    return "ACDC Vision Subsystem";
  }
}
