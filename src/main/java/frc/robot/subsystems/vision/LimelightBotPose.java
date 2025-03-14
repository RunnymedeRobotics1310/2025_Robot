package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Objects;

public class LimelightBotPose {

  private double[] botPose;
  private long timestampMicros;
  private double[] standardDeviations;

  /* Pose Data & Avg Tag Info */
  private static final int OFFSET_POSE_X = 0;
  private static final int OFFSET_POSE_Y = 1;
  private static final int OFFSET_POSE_Z = 2;
  private static final int OFFSET_POSE_ROTATION_ROLL = 3;
  private static final int OFFSET_POSE_ROTATION_PITCH = 4;
  private static final int OFFSET_POSE_ROTATION_YAW = 5;
  private static final int OFFSET_TOTAL_LATENCY = 6;
  private static final int OFFSET_TAG_COUNT = 7;
  private static final int OFFSET_TAG_SPAN = 8;
  private static final int OFFSET_AVG_TAG_DIST = 9;
  private static final int OFFSET_AVG_TAG_AREA = 10;

  /* Pose Data & Avg Tag Info */
  private static final int OFFSET_TAG_BASE = 11;
  private static final int ELEMENTS_PER_TAG = 7;
  private static final int OFFSET_TAG_ID = 0;
  private static final int OFFSET_TAG_TXNC = 1;
  private static final int OFFSET_TAG_TYNC = 2;
  private static final int OFFSET_TAG_TA = 3;
  private static final int OFFSET_TAG_DIST_TO_CAMERA = 4;
  private static final int OFFSET_TAG_DIST_TO_ROBOT = 5;
  private static final int OFFSET_TAG_AMBIGUITY = 6;

  public LimelightBotPose(double[] botPose, long timestampMicros, double[] standardDeviations) {
    update(botPose, timestampMicros, standardDeviations);
  }

  public void update(double[] botPose, long timestampMicros, double[] standardDeviations) {
    this.botPose = Objects.requireNonNullElseGet(botPose, () -> new double[0]);
    this.timestampMicros = timestampMicros;
    this.standardDeviations = standardDeviations;
  }

  public double[] getStandardDeviations() {
    return standardDeviations;
  }

  public Translation2d getTranslation() {
    return new Translation2d(getPoseX(), getPoseY());
  }

  public Pose2d getPose() {
    return new Pose2d(getTranslation(), Rotation2d.fromDegrees(getPoseRotationYaw()));
  }

  public double getPoseX() {
    return getElement(OFFSET_POSE_X);
  }

  public boolean isPoseXInBounds(double min, double max) {
    double poseX = getElement(OFFSET_POSE_X);
    return poseX >= min && poseX <= max;
  }

  public double getPoseY() {
    return getElement(OFFSET_POSE_Y);
  }

  public boolean isPoseYInBounds(double min, double max) {
    double poseY = getElement(OFFSET_POSE_Y);
    return poseY >= min && poseY <= max;
  }

  public double getPoseZ() {
    return getElement(OFFSET_POSE_Z);
  }

  public double getPoseRotationRoll() {
    return getElement(OFFSET_POSE_ROTATION_ROLL);
  }

  public double getPoseRotationPitch() {
    return getElement(OFFSET_POSE_ROTATION_PITCH);
  }

  public double getPoseRotationYaw() {
    return getElement(OFFSET_POSE_ROTATION_YAW);
  }

  public double getTotalLatency() {
    return getElement(OFFSET_TOTAL_LATENCY);
  }

  public double getTotalLatencySeconds() {
    return getElement(OFFSET_TOTAL_LATENCY) / 1000.0;
  }

  public double getTagCount() {
    return getElement(OFFSET_TAG_COUNT, 0);
  }

  public double getTagSpan() {
    return getElement(OFFSET_TAG_SPAN);
  }

  public double getAvgTagDist() {
    return getElement(OFFSET_AVG_TAG_DIST);
  }

  public double getAvgTagArea() {
    return getElement(OFFSET_AVG_TAG_AREA);
  }

  /**
   * Get the list of tags visible
   *
   * @return array list of tag ids visible
   */
  public double[] getVisibleTags() {
    ArrayList<Double> tags = new ArrayList<>();
    for (int i = 0; i < getTagCount(); i++) {
      tags.add(getTagId(i));
    }
    return tags.stream().mapToDouble(d -> d).toArray();
  }

  /**
   * Find the index of a tag in the bot pose data
   *
   * @param tagId the id of the tag to find
   * @return index of the tag in the bot pose data or -1 if not found
   */
  public int getTagIndex(int tagId) {
    for (int i = 0; i < getTagCount(); i++) {
      if (getTagId(i) == tagId) {
        return i;
      }
    }
    return -1;
  }

  public double getTagId(int index) {
    return getTagElement(index, OFFSET_TAG_ID);
  }

  public double getTagTxnc(int index) {
    return getTagElement(index, OFFSET_TAG_TXNC);
  }

  public double getTagTync(int index) {
    return getTagElement(index, OFFSET_TAG_TYNC);
  }

  public double getTagTa(int index) {
    return getTagElement(index, OFFSET_TAG_TA);
  }

  public double getTagDistToCamera(int index) {
    return getTagElement(index, OFFSET_TAG_DIST_TO_CAMERA);
  }

  public double getTagDistToRobot(int index) {
    return getTagElement(index, OFFSET_TAG_DIST_TO_ROBOT);
  }

  public double getTagAmbiguity(int index) {
    return getTagElement(index, OFFSET_TAG_AMBIGUITY);
  }

  public double getTimestampSeconds() {
    return timestampMicros / 1000000.0; // Convert from micros to seconds
  }

  private double getElement(int index) {
    return getElement(index, Double.MIN_VALUE);
  }

  private double getElement(int index, double defaultValue) {
    if (index < 0 || index >= botPose.length) {
      return defaultValue;
    }
    return botPose[index];
  }

  private double getTagElement(int index, int offset) {
    int indexCalc = OFFSET_TAG_BASE + (index * ELEMENTS_PER_TAG) + offset;
    if (index < 0 || index >= getTagCount() || indexCalc >= botPose.length) {
      return Double.MIN_VALUE;
    }
    return botPose[indexCalc];
  }
}
