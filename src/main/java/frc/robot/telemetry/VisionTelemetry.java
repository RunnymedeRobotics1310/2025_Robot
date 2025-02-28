/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package frc.robot.telemetry;

import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightPoseEstimate;

/**
 * @author JZ
 * @since 2025-02-16 10:45
 */
public class VisionTelemetry {

  /** Whether the vision telemetry is enabled */
  public boolean enabled = false;

  // Pose
  /** The x location of the robot with respect to the field in metres */
  public double poseMetresX = Double.MIN_VALUE;

  /** The y location of the robot with respect to the field in metres */
  public double poseMetresY = Double.MIN_VALUE;

  /** The heading of the robot with respect to the field in degrees */
  public double poseHeadingDegrees = Double.MIN_VALUE;

  /**
   * The x location of the robot with respect to the field as measured by the vision system in
   * metres
   */
  public double visionPoseX = Double.MIN_VALUE;

  /**
   * The y location of the robot with respect to the field as measured by the vision system in
   * metres
   */
  public double visionPoseY = Double.MIN_VALUE;

  /**
   * The heading of the robot with respect to the field as measured by the vision system in degrees
   */
  public double visionPoseHeading = Double.MIN_VALUE;

  /** The distance offset of vision pose to odometry */
  public double poseDeltaMetres = Double.MIN_VALUE;

  /** The confidence of the pose estimate */
  public LimelightPoseEstimate.PoseConfidence poseConfidence =
      LimelightPoseEstimate.PoseConfidence.NONE;

  /** The tag ambiguity of the currently in focus tag */
  public double tagAmbiguity = Double.MIN_VALUE;

  void post() {
    // Do nothing if not enabled
    if (!enabled) {
      return;
    }

    String poseOdo =
        String.format("(%.2f, %.2f) m %.1f deg", poseMetresX, poseMetresY, poseHeadingDegrees);
    SmartDashboard.putString(PREFIX + "Vision/pose_odo", poseOdo);

    String poseVis =
        String.format("(%.2f, %.2f) m %.1f deg", visionPoseX, visionPoseY, visionPoseHeading);
    SmartDashboard.putString(PREFIX + "Vision/pose_vis", poseVis);

    SmartDashboard.putNumber(PREFIX + "Vision/pose_delta", poseDeltaMetres);

    SmartDashboard.putString(PREFIX + "Vision/pose_confidence", poseConfidence.toString());

    SmartDashboard.putNumber(PREFIX + "Vision/tag_ambiguity", tagAmbiguity);
  }
}
