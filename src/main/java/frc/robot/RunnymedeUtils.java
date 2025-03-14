package frc.robot;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;

public class RunnymedeUtils {

  private static final long ALLIANCE_CACHE_TIME_MILLIS = 5000;
  private static DriverStation.Alliance alliance = null;
  private static long allianceLastUpdated = 0;

  /**
   * Get the active Alliance. This will return Red if there is no data from the FMS/DriverStation.
   * It will cache the value for 5 seconds, as when this is constantly used on the robot, it was
   * occupying close to 1% CPU. It does need to check periodically, as the alliance could
   * potentially change.
   *
   * @return current alliance from FMS/DS, or Red if none set
   */
  public static DriverStation.Alliance getRunnymedeAlliance() {
    long currentTime = System.currentTimeMillis();
    if (alliance == null || currentTime - allianceLastUpdated > ALLIANCE_CACHE_TIME_MILLIS) {
      DriverStation.getAlliance()
          .ifPresent(
              value -> {
                alliance = value;
                allianceLastUpdated = currentTime;
              });
    }

    return Objects.requireNonNullElse(alliance, DriverStation.Alliance.Red);
  }

  /**
   * Get the "red alliance version" of the specified location in the blue alliance. This is not a
   * coordinate system transformation - it is providing the coordinates (in "always blue alliance
   * origin" coordinates) of the corresponding field location in the red alliance of an object in
   * the blue alliance. E.g. given blue processor coordinates, return the red processor coordinates.
   *
   * <p>This assumes a rotated field, NOT a mirrored field.
   *
   * @see Constants#VISION_CONFIG#fieldExtentMetresX()
   * @see Constants#VISION_CONFIG#fieldExtentMetresY()
   * @param blueAllianceTranslation a location on the field with respect to the blue alliance
   * @return the corresponding location on the field with respect to the red alliance
   */
  public static Translation2d getRedAllianceLocation(Translation2d blueAllianceTranslation) {
    return new Translation2d(
        Constants.VISION_CONFIG.fieldExtentMetresX() - blueAllianceTranslation.getX(),
        Constants.VISION_CONFIG.fieldExtentMetresX() - blueAllianceTranslation.getY());
  }

  /**
   * Get the pose corresponding to the specified blue alliance location in the red alliance. The
   * heading is rotated 180 degrees.
   *
   * @see #getRedAllianceLocation(Translation2d)
   * @param blueAlliancePose a pose in the blue alliance
   * @return the corresponding pose in the red alliance
   */
  public static Pose2d getRedAlliancePose(Pose2d blueAlliancePose) {
    return new Pose2d(
        Constants.VISION_CONFIG.fieldExtentMetresX() - blueAlliancePose.getX(),
        Constants.VISION_CONFIG.fieldExtentMetresX() - -blueAlliancePose.getY(),
        Rotation2d.fromDegrees(
            SwerveUtils.normalizeDegrees(blueAlliancePose.getRotation().getDegrees() + 180)));
  }

  /**
   * Get the fastest possible speed to travel to the <code>distanceToTravel</code>, given the
   * specified parameters.
   *
   * <p>The distance can be provided in any unit - encoder counts, centimetres, degrees, etc. The
   * speed can also be provided in any unit.
   *
   * <p>The returned speed will accelerate from the <code>startSpeed</code> to the <code>maxSpeed
   * </code> over the <code>accelerationDistance</code>, then run at <code>maxSpeed</code>, then
   * decelerate to the <code>minSpeed</code> over the <code>decelerationDistance</code>.
   *
   * <p>If the distance is too short to reach cruising speed, then the <code>maxSpeed</code>, <code>
   * accelerationDistance</code> and <code>decelerationDistance</code> will all be reduced
   * proportionately so that the speed can be maximized while preserving the acceleration and
   * deceleration behaviour.
   *
   * <p>When the <code>distanceToTravel</code> reaches 0, the journey is complete and a speed of 0
   * is returned.
   *
   * @param distanceToTravel The distance that should be travelled. This must be a non-negative
   *     number
   * @param currentSpeed The current speed of the system. This number must be a non-negative number.
   * @param minSpeed The speed at the beginning of the acceleration period and at the end of the
   *     deceleration period. This must be a non-negative number.
   *     <p>It must be possible to start from 0 and immediately jump to the <code>minSpeed</code>,
   *     and likewise, it must be possible to start from the <code>minSpeed</code> and decelerate to
   *     0 immediately.
   * @param maxSpeed The target maximum speed to be reached during the cruise stage. This number
   *     must be greater than minSpeed
   * @param accelDistance the distance (in the same unit as the other distance parameters) over
   *     which the speed will increase up to the <code>maxSpeed</code>.
   *     <p>If the sum of <code>accelerationDistance</code> and <code>decelerationDistance</code>
   *     exceeds the <code>distanceToTravel</code>, the acceleration will be proportionately scaled
   *     down to ensure approximately the same acceleration curve.
   * @param decelDistance the distance (in the same unit as the other distance parameters) over
   *     which the speed will decrease down to the <code>minSpeed</code>
   *     <p>If the sum of <code>accelerationDistance</code> and <code>decelerationDistance</code>
   *     exceeds the <code>distanceToTravel</code>, the acceleration will be proportionately scaled
   *     down to ensure approximately the same acceleration curve.
   * @return the fastest speed within the bounds specified. If the <code>distanceToTravel</code>
   *     reaches 0, a speed of 0 will be returned.
   *     <p>Note that if the <code>maxSpeed</code> is greater than 0, then when the <code>
   *     distanceToTravel</code> is reached, the next speed returned will be 0. This could represent
   *     a discontinuity in speed by a using system if the returned value is not monitored. For
   *     example, if a system should continue to coast once the <code>distanceToTravel</code> is
   *     reached, the returned 0 value should be ignored.
   * @throws IllegalArgumentException if the input parameters are invalid
   */
  public static double calculateSpeed(
      double distanceToTravel,
      double currentSpeed,
      double minSpeed,
      double maxSpeed,
      double accelDistance,
      double decelDistance) {
    boolean DEBUG = false;
    if (distanceToTravel < 0)
      throw new IllegalArgumentException(
          "Distance to travel " + distanceToTravel + " must not be negative");
    if (currentSpeed < 0) {
      throw new IllegalArgumentException("Current speed " + currentSpeed + " must not be negative");
    }
    if (minSpeed < 0)
      throw new IllegalArgumentException("Min speed " + minSpeed + " must not be negative");
    if (maxSpeed < 0)
      throw new IllegalArgumentException("Max speed " + maxSpeed + " must not be negative");
    if (maxSpeed < minSpeed)
      throw new IllegalArgumentException(
          "Min speed " + minSpeed + " must not exceed max speed " + maxSpeed);
    if (accelDistance < 0) {
      throw new IllegalArgumentException(
          "Acceleration distance " + accelDistance + " must not be negative");
    }
    if (decelDistance < 0) {
      throw new IllegalArgumentException(
          "Deceleration distance " + decelDistance + " must not be negative");
    }

    if (distanceToTravel < 1e-3) {
      return 0;
    }

    enum Zone {
      ACCEL,
      CRUISE,
      DECEL
    }

    // figure out zone
    final Zone zone;
    final double distanceToTravelInAccelZone;

    if (distanceToTravel < decelDistance) {
      zone = Zone.DECEL;
      distanceToTravelInAccelZone = 0;
    } else {
      // how far are we from max?
      double pctMax = currentSpeed / maxSpeed;
      // we are part way to max, so we only care about the rest of the distance to max
      if (pctMax < 1) {
        accelDistance *= (1 - pctMax);
      }

      if (distanceToTravel > accelDistance + decelDistance) {
        // we will get to max speed
        if (currentSpeed < maxSpeed) {
          zone = Zone.ACCEL;
          distanceToTravelInAccelZone = accelDistance;
        } else {
          zone = Zone.CRUISE;
          distanceToTravelInAccelZone = 0;
        }
      } else {
        // we cannot get to max speed - scale it down
        maxSpeed = (distanceToTravel / (accelDistance + decelDistance)) * maxSpeed;
        if (DEBUG) System.out.println("SCALE DOWN - new max: " + String.format("%.2f", maxSpeed));
        if (currentSpeed < maxSpeed) {
          zone = Zone.ACCEL;
          distanceToTravelInAccelZone = accelDistance;
        } else {
          zone = Zone.CRUISE;
          distanceToTravelInAccelZone = 0;
        }
      }
    }

    if (DEBUG)
      System.out.println("Distance to travel in accel zone: " + distanceToTravelInAccelZone);
    final double FACTOR = 5;

    final double speed;
    switch (zone) {
      case CRUISE:
        {
          double delta = currentSpeed - maxSpeed;
          if (Math.abs(delta) < 1e-6) {
            if (DEBUG) System.out.println("CRUISE: at max speed");
            speed = maxSpeed;
          } else if (delta > 0) {
            // too fast! slow down ASAP but not discontinuously
            if (DEBUG)
              System.out.println(
                  "CRUISE: too fast! slow down ASAP but not discontinuously - can't do target "
                      + String.format("%.2f", maxSpeed));
            ;
            speed = Math.max(currentSpeed - (delta / FACTOR), maxSpeed); // todo: smooth this
          } else {
            // too slow! speed up ASAP but not discontinuously
            if (DEBUG)
              System.out.println(
                  "CRUISE: too slow! speed up ASAP but not discontinuously - can't do target "
                      + String.format("%.2f", maxSpeed));
            speed = Math.min(currentSpeed + (-delta / FACTOR), maxSpeed); // todo: smooth this
          }
          break;
        }
      case ACCEL:
        {
          double target =
              maxSpeedAccelZone(minSpeed, maxSpeed, distanceToTravelInAccelZone, accelDistance);
          double delta = currentSpeed - target;
          if (Math.abs(delta) < 1e-6) {
            speed = target;
            if (DEBUG) System.out.println("ACCEL: at target " + String.format("%.2f", target));
          } else if (delta > 0) {
            // too fast! slow down ASAP but not discontinuously
            if (DEBUG)
              System.out.println(
                  "ACCEL: too fast! slow down ASAP but not discontinuously - can't do target "
                      + String.format("%.2f", target));
            speed = Math.max(currentSpeed - (delta / FACTOR), target); // todo: smooth this
          } else {
            // too slow! speed up ASAP but not discontinuously
            if (DEBUG)
              System.out.println(
                  "ACCEL: too slow! speed up ASAP but not discontinuously - can't do target "
                      + String.format("%.2f", target));
            speed = Math.min(currentSpeed + (-delta / FACTOR), target); // todo: smooth this
          }
          break;
        }
      case DECEL:
        {
          // define the curve
          double target = maxSpeedDecelZone(minSpeed, maxSpeed, distanceToTravel, decelDistance);
          double delta = currentSpeed - target;
          if (Math.abs(delta) < 1e-6) {
            speed = target;
            if (DEBUG) System.out.println("DECEL: at target " + String.format("%.2f", target));
          } else if (delta > 0) {
            // too fast! slow down ASAP but not discontinuously
            if (DEBUG)
              System.out.println(
                  "DECEL: too fast! slow down ASAP but not discontinuously - can't do target "
                      + String.format("%.2f", target));
            speed = Math.max(currentSpeed - (delta / FACTOR), target); // todo: smooth this
          } else {
            // too slow! speed up ASAP but not discontinuously
            if (DEBUG)
              System.out.println(
                  "DECEL: too slow! speed up ASAP but not discontinuously - can't do target "
                      + String.format("%.2f", target));
            speed = Math.min(currentSpeed + (-delta / FACTOR), target); // todo: smooth this
          }
          break;
        }
      default:
        {
          throw new IllegalStateException("Unknown zone: " + zone);
        }
    }

    return speed;
  }

  static double maxSpeedDecelZone(
      double minSpeed, double maxSpeed, double distanceToTravelInDecelZone, double decelDistance) {
    if (distanceToTravelInDecelZone > decelDistance)
      throw new IllegalArgumentException(
          "Distance to travel in deceleration zone ("
              + String.format("%.2f", distanceToTravelInDecelZone)
              + ") cannot exceed the decleeration distance ("
              + String.format("%.2f", decelDistance)
              + ")");
    return (((minSpeed - maxSpeed) / decelDistance) * distanceToTravelInDecelZone) + maxSpeed;
  }

  static double maxSpeedAccelZone(
      double minSpeed, double maxSpeed, double distanceToTravelInAccelZone, double accelDistance) {
    if (distanceToTravelInAccelZone > accelDistance)
      throw new IllegalArgumentException(
          "Distance to travel in acceleration zone ("
              + String.format("%.2f", distanceToTravelInAccelZone)
              + ") cannot exceed the decleeration distance ("
              + String.format("%.2f", accelDistance)
              + ")");
    double m = (maxSpeed - minSpeed) / accelDistance;
    return (m * distanceToTravelInAccelZone) + minSpeed;
  }
}
