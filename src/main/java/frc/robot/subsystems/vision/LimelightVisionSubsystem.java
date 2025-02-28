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

  private final NetworkTable lowRiderVision =
      NetworkTableInstance.getDefault().getTable("limelight-nikola");
  private final NetworkTable elevateVision =
      NetworkTableInstance.getDefault().getTable("limelight-thomas");

  // inputs/configs
  private final NetworkTableEntry lr_camMode = lowRiderVision.getEntry("camMode");
  private final NetworkTableEntry lr_pipeline = lowRiderVision.getEntry("pipeline");
  private final DoubleArrayPublisher lr_robotOrientation =
      lowRiderVision.getDoubleArrayTopic("robot_orientation_set").publish();
  private final DoubleArrayPublisher thomas_cameraLocation =
       elevateVision.getDoubleArrayTopic("camerapose_robotspace_set").publish();


  private final NetworkTableEntry elevate_camMode = lowRiderVision.getEntry("camMode");
  private final NetworkTableEntry elevate_pipeline = lowRiderVision.getEntry("pipeline");
  private final DoubleEntry elevate_stream = elevateVision.getDoubleTopic("stream").getEntry(-1);

  // output
  private final DoubleArraySubscriber lr_MegaTag1 =
      lowRiderVision.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  private final DoubleArraySubscriber lr_MegaTag2 =
      lowRiderVision.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);

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

  private Matrix<N3, N1> poseDeviationMegaTag1 = VecBuilder.fill(0.01, 0.01, 0.05);
  private Matrix<N3, N1> poseDeviationMegaTag2 = VecBuilder.fill(0.06, 0.06, 9999999);

  private final LimelightBotPose limelightBotPose = new LimelightBotPose(null, 0);
  LimelightPoseEstimate currentPoseEstimate =
      new LimelightPoseEstimate(
          limelightBotPose.getPose(),
          limelightBotPose.getTimestampSeconds(),
          poseDeviationMegaTag1,
          LimelightPoseEstimate.PoseConfidence.NONE);

  private double[] orientationSet = new double[] {0, 0, 0, 0, 0, 0};

  private double[] thomasPositionSet = new double[] {0, 0, 0, 0, 0, 0};

  public void setThomasHeight(double height){
    thomasPositionSet[2] = height;
    thomas_cameraLocation.set(thomasPositionSet);
  }

  private final double fieldExtentMetresX;
  private final double fieldExtentMetresY;
  private final double maxAmbiguity;
  private final double highQualityAmbiguity;
  private final double maxVisposDeltaDistanceMetres;

  private int targetTagId = 0;

  public LimelightVisionSubsystem(VisionConfig visionConfig) {
    this.fieldExtentMetresX = visionConfig.fieldExtentMetresX();
    this.fieldExtentMetresY = visionConfig.fieldExtentMetresY();
    this.maxAmbiguity = visionConfig.maxAmbiguity();
    this.highQualityAmbiguity = visionConfig.highQualityAmbiguity();
    this.maxVisposDeltaDistanceMetres = visionConfig.maxVisposeDeltaDistanceMetres();

    this.lr_pipeline.setNumber(visionConfig.pipelineAprilTagDetect());
    this.lr_camMode.setNumber(visionConfig.camModeVision());
    this.elevate_pipeline.setNumber(visionConfig.pipelineAprilTagDetect());
    this.elevate_camMode.setNumber(visionConfig.camModeVision());

    Telemetry.vision.enabled = visionConfig.telemetryEnabled();
  }

  @Override
  public void periodic() {
    TimestampedDoubleArray botPoseBlueMegaTag1 = lr_MegaTag1.getAtomic();
    limelightBotPose.update(botPoseBlueMegaTag1.value, botPoseBlueMegaTag1.timestamp);
    currentPoseEstimate.setPose(limelightBotPose.getPose());
    currentPoseEstimate.setTimestamp(limelightBotPose.getTimestampSeconds());
  }

  /* Public API */

  public void setTargetTagId(TagType tag) {
    this.targetTagId = tag.getIndex();
  }

  public void clearTargetTagId() {
    this.targetTagId = 0;
  }

  public double getVisibleTargetTagId() {
    return limelightBotPose.getTagId(0);
  }

  public double distanceToTarget() {
    int index = 0;
    if (targetTagId > 0) {
      index = limelightBotPose.getTagIndex(targetTagId);
    }
    return limelightBotPose.getTagDistToRobot(index);
  }

  public double angleToTarget() {
    int index = 0;
    if (targetTagId > 0) {
      index = limelightBotPose.getTagIndex(targetTagId);
    }
    return limelightBotPose.getTagTxnc(index);
  }

  /**
   * Set the camera view to the specified stream.
   *
   * @param stream the camera stream to set the view to
   */
  public void setCameraView(CamStreamType stream) {
    elevate_stream.set(stream.getIndex());
  }

  public LimelightBotPose getBotPose() {
    return limelightBotPose;
  }

  /**
   * Get the pose estimate from the vision system, for callback via VisionPoseCallback
   *
   * @param yaw the current yaw of the robot
   * @param yawRate the current yaw rate of the robot
   * @return the pose estimate
   */
  public PoseEstimate getPoseEstimate(Pose2d odometryPose, double yaw, double yawRate) {
    // First, update the limelight and let it know our orientation
    orientationSet[0] = yaw;
    lr_robotOrientation.set(orientationSet);

    PoseEstimate returnVal = null;

    // Get the current pose delta
    double compareDistance = -1;
    double compareHeading = -1;
    double tagAmbiguity = -1;

    // Telemetry Handling
    if (Telemetry.vision.enabled) {
      Telemetry.vision.poseMetresX = odometryPose.getX();
      Telemetry.vision.poseMetresY = odometryPose.getY();
      Telemetry.vision.poseHeadingDegrees = odometryPose.getRotation().getDegrees();
      Telemetry.vision.visionPoseX = currentPoseEstimate.getPose().getX();
      Telemetry.vision.visionPoseY = currentPoseEstimate.getPose().getY();
      Telemetry.vision.visionPoseHeading = currentPoseEstimate.getPose().getRotation().getDegrees();
    }

    // Reset some values in PoseEstimate
    currentPoseEstimate.setPoseConfidence(LimelightPoseEstimate.PoseConfidence.NONE);

    // If pose is 0,0 or no tags in view, we don't actually have data - return null
    if (currentPoseEstimate.getPose().getX() > 0
        && currentPoseEstimate.getPose().getY() > 0
        && limelightBotPose.getTagCount() > 0
        && currentPoseEstimate.getPose().getX() < fieldExtentMetresX
        && currentPoseEstimate.getPose().getY() < fieldExtentMetresY
        && yawRate <= 720) {

      // Get the "best" tag - assuming the first one is the best - TBD TODO
      tagAmbiguity = limelightBotPose.getTagAmbiguity(0);

      compareDistance =
          currentPoseEstimate.getPose().getTranslation().getDistance(odometryPose.getTranslation());
      compareHeading = currentPoseEstimate.getPose().getRotation().getDegrees() - yaw;

      if (tagAmbiguity < maxAmbiguity) {
        // If the ambiguity is very low, use the data as is (or when disabled, to allow for
        // bot repositioning
        if (tagAmbiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
          currentPoseEstimate.setStandardDeviations(poseDeviationMegaTag1);
          currentPoseEstimate.setPoseConfidence(LimelightPoseEstimate.PoseConfidence.HIGH);
          returnVal = currentPoseEstimate;
        } else {
          // We need to be careful with this data set. If the location is too far off,
          // don't use it. Otherwise, scale confidence by distance.
          if (compareDistance < maxVisposDeltaDistanceMetres) {
            double stdDevRatio = Math.pow(limelightBotPose.getTagDistToCamera(0), 2) / 2;
            double stdDevRatio2 = 5 * stdDevRatio;

            Matrix<N3, N1> deviation = VecBuilder.fill(stdDevRatio, stdDevRatio, stdDevRatio2);
            currentPoseEstimate.setStandardDeviations(deviation);
            currentPoseEstimate.setPoseConfidence(LimelightPoseEstimate.PoseConfidence.MEDIUM);
            returnVal = currentPoseEstimate;
          } else {
            currentPoseEstimate.setPoseConfidence(LimelightPoseEstimate.PoseConfidence.LOW);
          }
        }
      }
    }

    if (Telemetry.vision.enabled) {
      Telemetry.vision.tagAmbiguity = tagAmbiguity;
      Telemetry.vision.poseConfidence = currentPoseEstimate.getPoseConfidence();
      Telemetry.vision.poseDeltaMetres = compareDistance;
      Telemetry.vision.headingDeltaDegrees = compareHeading;
    }

    return returnVal;
  }

  @Override
  public String toString() {
    return "ACDC Vision Subsystem";
  }
}
