package frc.robot.subsystems.vision;

import ca.team1310.swerve.vision.PoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class LimelightPoseEstimate implements PoseEstimate {

  public enum PoseConfidence {
    NONE,
    MEGATAG1,
    MEGATAG2
  }

  private Pose2d pose;
  private double timestamp;
  private Matrix<N3, N1> standardDeviations;
  private PoseConfidence poseConfidence = PoseConfidence.NONE;

  public LimelightPoseEstimate(
      Pose2d pose,
      double timestamp,
      Matrix<N3, N1> standardDeviations,
      PoseConfidence poseConfidence) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.standardDeviations = standardDeviations;
    this.poseConfidence = poseConfidence;
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
  public Matrix<N3, N1> getStandardDeviations() {
    return standardDeviations;
  }

  public PoseConfidence getPoseConfidence() {
    return poseConfidence;
  }

  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  public void setTimestamp(double timestamp) {
    this.timestamp = timestamp;
  }

  public void setStandardDeviations(Matrix<N3, N1> standardDeviations) {
    this.standardDeviations = standardDeviations;
  }

  public void setPoseConfidence(PoseConfidence poseConfidence) {
    this.poseConfidence = poseConfidence;
  }
}
