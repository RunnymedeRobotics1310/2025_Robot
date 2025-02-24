/*
 * Copyright 2025 The Kingsway Digital Company Limited. All rights reserved.
 */
package frc.robot.telemetry;

import static frc.robot.telemetry.Telemetry.PREFIX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Tony Field
 * @since 2025-02-16 10:45
 */
public class DriveTelemetry {

  /** The x velocity of the robot with respect to the field in metres per second */
  public double fieldOrientedVelocityX = Double.MIN_VALUE;

  /** The y velocity of the robot with respect to the field in metres per second */
  public double fieldOrientedVelocityY = Double.MIN_VALUE;

  /** The rotational velocity of the robot with respect to the field in radians per second */
  public double fieldOrientedVelocityOmega = Double.MIN_VALUE;

  /**
   * The x distance in metres from the current robot location to the desired pose. Not set by
   * RunnymedeSwerve - typically set by subsystems that use RunnymedeSwerve.
   */
  public double fieldOrientedDeltaToPoseX = Double.MIN_VALUE;

  /**
   * The y distance in metres from the current robot location to the desired pose. Not set by
   * RunnymedeSwerve - typically set by subsystems that use RunnymedeSwerve.
   */
  public double fieldOrientedDeltaToPoseY = Double.MIN_VALUE;

  /**
   * The heading in degrees from the current robot location to the desired pose. Not set by
   * RunnymedeSwerve - typically set by subsystems that use RunnymedeSwerve.
   */
  public double fieldOrientedDeltaToPoseHeading = Double.MIN_VALUE;

  void post() {
    double fieldSpeed = Math.hypot(fieldOrientedVelocityX, fieldOrientedVelocityY);
    String vField =
        String.format(
            "%.1f (%.1f, %.1f) m/s %.1f deg/s",
            fieldSpeed,
            fieldOrientedVelocityX,
            fieldOrientedVelocityY,
            Math.toDegrees(fieldOrientedVelocityOmega));
    SmartDashboard.putString(PREFIX + "Drive/desired_velocity_field", vField);

    double dist = Math.hypot(fieldOrientedDeltaToPoseX, fieldOrientedDeltaToPoseY);
    String delta =
        String.format(
            "%.2f (%.2f, %.2f) m %.1f deg",
            dist,
            fieldOrientedDeltaToPoseX,
            fieldOrientedDeltaToPoseY,
            fieldOrientedDeltaToPoseHeading);
    SmartDashboard.putString(PREFIX + "Drive/distance_to_pose", delta);
  }
}
