package frc.robot.subsystems.vision;

import ca.team1310.swerve.vision.VisionPoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class LimelightPoseEstimate implements VisionPoseEstimate {

  public enum PoseConfidence {
    NONE,
    MEGATAG1_HIGH,
    MEGATAG1_MED,
    MEGATAG2
  }

  private final Pose2d pose;
  private final double timestamp;
  private final Matrix<N3, N1> standardDeviations;

  public LimelightPoseEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviations) {
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
  public Matrix<N3, N1> getStandardDeviations() {
    return standardDeviations;
  }
}
