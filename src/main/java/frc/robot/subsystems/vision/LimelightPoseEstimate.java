package frc.robot.subsystems.vision;

import ca.team1310.swerve.vision.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;

public class LimelightPoseEstimate implements PoseEstimate {

  public enum PoseConfidence {
    NONE,
    MEGATAG1,
    MEGATAG2
  }

  private Pose2d pose;
  private double timestamp;
  private double[] standardDeviations;

  public LimelightPoseEstimate(Pose2d pose, double timestamp, double[] standardDeviations) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.standardDeviations = standardDeviations;
  }

  @Override
  public Pose2d getPose() {
    return pose;
  }

  @Override
  public double getTimestampSeconds() {
    return timestamp;
  }

  @Override
  public double[] getStandardDeviations() {
    return standardDeviations;
  }

  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  public void setTimestamp(double timestamp) {
    this.timestamp = timestamp;
  }

  public void setStandardDeviations(double[] standardDeviations) {
    this.standardDeviations = standardDeviations;
  }
}
