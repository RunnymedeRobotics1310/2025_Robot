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
import edu.wpi.first.networktables.DoubleEntry;
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
  // TODO: Remove properly
  //  private final DoubleArrayPublisher thomasCameraLocation =
  //      thomasVision.getDoubleArrayTopic("camerapose_robotspace_set").publish();

  private final NetworkTableEntry thomasCamMode = thomasVision.getEntry("camMode");
  private final NetworkTableEntry thomsPipeline = thomasVision.getEntry("pipeline");
  private final DoubleArrayPublisher thomasRobotOrientation =
      thomasVision.getDoubleArrayTopic("robot_orientation_set").publish();
  private final DoubleEntry thomasStream = thomasVision.getDoubleTopic("stream").getEntry(-1);

  // MegaTags
  private final DoubleArraySubscriber nikolaMegaTag1 =
      nikolaVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  private final DoubleArraySubscriber nikolaStddevs =
      nikolaVision.getDoubleArrayTopic("stddevs").subscribe(new double[0]);
  private final DoubleArraySubscriber thomasMegaTag1 =
      thomasVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  private final DoubleArraySubscriber thomasStddevs =
      thomasVision.getDoubleArrayTopic("stddevs").subscribe(new double[0]);

  //  private final DoubleArraySubscriber nikolaMegaTag2 =
  //          nikolaVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

  public enum CamStreamType {
    SIDE_BY_SIDE(0),
    LIMELIGHT(1),
    WEBCAM(2);

    private final int index;

    // Constructor for the enum, which assigns the index to each constant.
    private CamStreamType(int index) {
      this.index = index;
    }

    // Getter method to retrieve the index of the enum constant.
    public int getIndex() {
      return index;
    }
  }

  public enum TagType {
    RED_SOURCE_LEFT(1),
    RED_SOURCE_RIGHT(2),
    RED_PROCESSOR(3),
    RED_REEF_1(6),
    RED_REEF_2(7),
    RED_REEF_3(8),
    RED_REEF_4(9),
    RED_REEF_5(10),
    RED_REEF_6(11),
    BLUE_SOURCE_LEFT(12),
    BLUE_SOURCE_RIGHT(13),
    BLUE_PROCESSOR(16),
    BLUE_REEF_1(17),
    BLUE_REEF_2(18),
    BLUE_REEF_3(19),
    BLUE_REEF_4(20),
    BLUE_REEF_5(21),
    BLUE_REEF_6(22);

    private int tag;

    // Constructor for the enum, which assigns the index to each constant.
    private TagType(int tag) {
      this.tag = tag;
    }

    // Getter method to retrieve the index of the enum constant.
    public int getIndex() {
      return tag;
    }
  }

  private static final Matrix<N3, N1> POSE_DEVIATION_MEGATAG1 = VecBuilder.fill(0.01, 0.01, 0.05);
  private static final Matrix<N3, N1> POSE_DEVIATION_MEGATAG2 =
      VecBuilder.fill(0.06, 0.06, 9999999);

  private final LimelightBotPose nikolaBotPoseMegaTag1 = new LimelightBotPose(null, 0, null);
  private final LimelightBotPose thomasBotPoseMegaTag1 = new LimelightBotPose(null, 0, null);

  private double[] orientationSet = new double[] {0, 0, 0, 0, 0, 0};
  private double[] thomasPositionSet = new double[] {0.155, 0.13, 0.88, 179, 0, 0};

  private final double fieldExtentMetresX;
  private final double fieldExtentMetresY;
  private final double maxAmbiguity;
  private final double highQualityAmbiguity;
  private final double maxVisposDeltaDistanceMetres;

  private boolean poseUpdatesEnabled = true;

  private int targetTagId = 0;

  public LimelightVisionSubsystem(VisionConfig visionConfig) {
    this.fieldExtentMetresX = visionConfig.fieldExtentMetresX();
    this.fieldExtentMetresY = visionConfig.fieldExtentMetresY();
    this.maxAmbiguity = visionConfig.maxAmbiguity();
    this.highQualityAmbiguity = visionConfig.highQualityAmbiguity();
    this.maxVisposDeltaDistanceMetres = visionConfig.maxVisposeDeltaDistanceMetres();

    this.nikolaPipeline.setNumber(visionConfig.pipelineAprilTagDetect());
    this.nikolaCamMode.setNumber(visionConfig.camModeVision());
    this.thomsPipeline.setNumber(visionConfig.pipelineAprilTagDetect());
    this.thomasCamMode.setNumber(visionConfig.camModeVision());

    Telemetry.vision.enabled = visionConfig.telemetryEnabled();
  }

  @Override
  public void periodic() {
    TimestampedDoubleArray nikolaBotPoseBlueMegaTag1 = nikolaMegaTag1.getAtomic();
    double[] nikolaStddevData = nikolaStddevs.get();
    nikolaBotPoseMegaTag1.update(
        nikolaBotPoseBlueMegaTag1.value, nikolaBotPoseBlueMegaTag1.timestamp, nikolaStddevData);

    TimestampedDoubleArray thomasBotPoseBlueMegaTag1 = thomasMegaTag1.getAtomic();
    double[] thomasStddevData = thomasStddevs.get();
    thomasBotPoseMegaTag1.update(
        thomasBotPoseBlueMegaTag1.value, thomasBotPoseBlueMegaTag1.timestamp, thomasStddevData);
  }

  /** If targetting the left reef, use the left reef pose, otherwise use the right reef pose */
  private LimelightBotPose getBotPose(boolean leftBranch) {
    return leftBranch ? nikolaBotPoseMegaTag1 : thomasBotPoseMegaTag1;
  }

  /* Public API */

  public void setPoseUpdatesEnabled(boolean enabled) {
    poseUpdatesEnabled = enabled;
  }

  public void setThomasHeight(double height) {
    thomasPositionSet[2] = height;
    //    thomasCameraLocation.set(thomasPositionSet);  // TODO: Remove properly
  }

  public void setTargetTagId(TagType tag) {
    this.targetTagId = tag.getIndex();
  }

  public void setTargetTagId(int tagId) {
    this.targetTagId = tagId;
  }

  public void clearTargetTagId() {
    this.targetTagId = 0;
  }

  public double getVisibleTargetTagId() {
    return nikolaBotPoseMegaTag1.getTagId(0);
  }

  public double getVisibleTargetTagId(boolean leftBranch) {
    return getBotPose(leftBranch).getTagId(0);
  }

  public int getNumTagsVisible() {
    return (int) nikolaBotPoseMegaTag1.getTagCount();
  }

  public double distanceTagToRobot() {
    return distanceTagToRobot(true);
  }

  public double distanceTagToRobot(boolean leftBranch) {
    LimelightBotPose botPose = getBotPose(leftBranch);

    int index = 0;
    if (targetTagId > 0) {
      index = botPose.getTagIndex(targetTagId);
    }
    return botPose.getTagDistToRobot(index);
  }

  public double distanceTagToCamera() {
    return distanceTagToCamera(true);
  }

  public double distanceTagToCamera(boolean leftBranch) {
    LimelightBotPose botPose = getBotPose(leftBranch);

    int index = 0;
    if (targetTagId > 0) {
      index = botPose.getTagIndex(targetTagId);
    }
    return botPose.getTagDistToCamera(index);
  }

  public double angleToTarget() {
    return angleToTarget(true);
  }

  public double angleToTarget(boolean leftBranch) {
    LimelightBotPose botPose = getBotPose(leftBranch);

    int index = 0;
    if (targetTagId > 0) {
      index = botPose.getTagIndex(targetTagId);
    }
    return -botPose.getTagTxnc(index);
  }

  public double getTagAmount() {
    return nikolaBotPoseMegaTag1.getTagCount();
  }

  public boolean isTagInView(int tagId) {
    return nikolaBotPoseMegaTag1.getTagIndex(tagId) != -1;
  }

  /**
   * Set the camera view to the specified stream.
   *
   * @param stream the camera stream to set the view to
   */
  public void setCameraView(CamStreamType stream) {
    // thomasStream.set(stream.getIndex());  //TODO: Remove properly when not needed
  }

  public LimelightBotPose getBotPose() {
    return nikolaBotPoseMegaTag1;
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
   *   <li>Check tag ambiguity - if it's very low (<0.1) it means we've got a very very high
   *       confidence set of data, and we therefore use MegaTag1 data to update pose and heading
   *       data, which will keep MegaTag 2 honest.
   *   <li>If ambiguity is medium (0.1-0.7), we can trust MegaTag2 as a high confidence source
   *   <li>Otherwise, let's not use high ambiguity data at all and let Odometry do its thing
   * </ol>
   *
   * @param odometryPose the current odometry pose of the robot
   * @param yaw the current yaw of the robot
   * @param yawRate the current yaw rate of the robot
   * @return the pose estimate
   */
  public PoseEstimate getPoseEstimate(Pose2d odometryPose, double yaw, double yawRate) {

    // First, update the limelight and let it know our orientation
    orientationSet[0] = yaw;
    nikolaRobotOrientation.set(orientationSet);
    thomasRobotOrientation.set(orientationSet);

    PoseEstimate returnVal = null;

    // Get the current pose delta
    double compareDistance = -1;
    double compareHeading = -1;
    double tagAmbiguity = nikolaBotPoseMegaTag1.getTagAmbiguity(0);
    double[] deviations = new double[] {-1, -1, -1};

    LimelightPoseEstimate.PoseConfidence poseConfidence = LimelightPoseEstimate.PoseConfidence.NONE;
    LimelightBotPose botPose = nikolaBotPoseMegaTag1; // default to MT1 for telemetry

    // If pose is 0,0 or no tags in view, we don't actually have data - return null
    if (nikolaBotPoseMegaTag1.getTagCount() > 0
        && nikolaBotPoseMegaTag1.isPoseXInBounds(0, fieldExtentMetresX)
        && nikolaBotPoseMegaTag1.isPoseYInBounds(0, fieldExtentMetresY)) {

      // Do we have a decent signal?  i.e. Ambiguity < 0.7
      if (tagAmbiguity < maxAmbiguity) {
        // Check for super good signal - ambiguity < 0.1, or we're disabled (field setup)
        if (tagAmbiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
          // use megatag1 as is, it's rock solid
          double[] stddevs = nikolaBotPoseMegaTag1.getStandardDeviations();
          deviations[0] = stddevs[0]; // MegaTag1 X Standard Deviation
          deviations[1] = stddevs[1]; // MegaTag1 Y Standard Deviation
          deviations[2] = stddevs[5]; // MegaTag Deg Standard Deviation
          poseConfidence = LimelightPoseEstimate.PoseConfidence.MEGATAG1;

          returnVal =
              new LimelightPoseEstimate(
                  nikolaBotPoseMegaTag1.getPose(),
                  nikolaBotPoseMegaTag1.getTimestampSeconds(),
                  deviations);
        }

        if (Telemetry.vision.enabled) {
          compareDistance =
              botPose.getPose().getTranslation().getDistance(odometryPose.getTranslation());
          compareHeading =
              botPose.getPose().getRotation().getDegrees()
                  - odometryPose.getRotation().getDegrees();
        }
      }
    }

    // Telemetry Handling
    if (Telemetry.vision.enabled) {
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
      Telemetry.vision.standardDeviations = deviations;
      Telemetry.vision.navxYaw = yaw;
      Telemetry.vision.navxYawDelta = odometryPose.getRotation().getDegrees() - yaw;
      Telemetry.vision.poseXSeries.add(botPose.getPoseX());
      Telemetry.vision.poseYSeries.add(botPose.getPoseY());
      Telemetry.vision.poseDegSeries.add(botPose.getPoseRotationYaw());

      Telemetry.vision.nikVisibleTags = nikolaBotPoseMegaTag1.getVisibleTags();
      Telemetry.vision.nikTx = nikolaBotPoseMegaTag1.getTagTxnc(0);
      Telemetry.vision.nikDistanceToRobot = nikolaBotPoseMegaTag1.getTagDistToRobot(0);
      Telemetry.vision.nikDistanceToCam = nikolaBotPoseMegaTag1.getTagDistToCamera(0);

      Telemetry.vision.tomVisibleTags = thomasBotPoseMegaTag1.getVisibleTags();
      Telemetry.vision.tomTx = thomasBotPoseMegaTag1.getTagTxnc(0);
      Telemetry.vision.tomDistanceToRobot = thomasBotPoseMegaTag1.getTagDistToRobot(0);
      Telemetry.vision.tomDistanceToCam = thomasBotPoseMegaTag1.getTagDistToCamera(0);
    }

    return poseUpdatesEnabled ? returnVal : null;
  }

  @Override
  public String toString() {
    return "ACDC Vision Subsystem";
  }
}
