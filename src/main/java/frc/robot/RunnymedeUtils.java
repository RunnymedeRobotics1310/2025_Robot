package frc.robot;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;

public class RunnymedeUtils {

  private static final long ALLIANCE_CACHE_TIME_MILLIS = 5000;
  private static DriverStation.Alliance alliance = null;
  private static long allianceLastUpdated = 0;
  private static double teleOpMatchStartTime = 0;

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
   * Sets the timestamp Teleop started for match timing purposes
   *
   * @param seconds From Timer.getFPGATimestamp()
   */
  public static void setTeleopMatchStartTime(double seconds) {
    teleOpMatchStartTime = seconds;
  }

  /**
   * How much time is remaining in Teleop for a match.
   *
   * @return Seconds if there's time remaining, 0 if we're overtime.
   */
  public static double teleopMatchTimeRemaining() {
    double remainingTime = Timer.getFPGATimestamp() - teleOpMatchStartTime;

    // If match time is within 2m15s, return it.  Otherwise, return 0.
    return (135 - remainingTime) >= 0 ? 135 - remainingTime : -1;
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
        Constants.FieldConstants.FIELD_EXTENT_METRES_X - blueAllianceTranslation.getX(),
        Constants.FieldConstants.FIELD_EXTENT_METRES_Y - blueAllianceTranslation.getY());
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
        Constants.FieldConstants.FIELD_EXTENT_METRES_X - blueAlliancePose.getX(),
        Constants.FieldConstants.FIELD_EXTENT_METRES_Y - blueAlliancePose.getY(),
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
    boolean DEBUG = true;
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

    final double requestedAcceleration = (maxSpeed - minSpeed) / accelDistance;
    final double requestedDeceleration = (minSpeed - maxSpeed) / decelDistance;

    // figure out zone
    final Zone zone;
    final double distanceToTravelInAccelZone;

    // our current speed affects how much space we need to accel or decel.

    // how far are we from min?
    double pctMin = currentSpeed / minSpeed;
    // we are part way to min, so we only care about the rest of the distance to min
    if (pctMin < 1) {
      decelDistance *= (1 - pctMin);
    }

    // how far are we from max?
    double pctMax = currentSpeed / maxSpeed;
    // we are part way to max, so we only care about the rest of the distance to max
    if (pctMax < 1) {
      accelDistance *= (1 - pctMax);
    }

    // given current location, speed, and deceleration distance, are we in the deceleration zone?
    if (distanceToTravel < decelDistance) {
      zone = Zone.DECEL;
      distanceToTravelInAccelZone = 0;
    } else {

      // we are either in the cruise zone or the acceleration zone
      if (distanceToTravel >= accelDistance + decelDistance) {
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
        double newMax = (distanceToTravel / (accelDistance + decelDistance)) * maxSpeed;
        if (DEBUG)
          System.out.printf(
              "SCALE DOWN not enough room to reach speed - old max %.2f, new max %.2f\n",
              maxSpeed, newMax);
        maxSpeed = newMax;
        if (currentSpeed < maxSpeed) {
          zone = Zone.ACCEL;
          distanceToTravelInAccelZone = accelDistance;
        } else {
          zone = Zone.CRUISE;
          distanceToTravelInAccelZone = 0;
        }
      }
    }

    final double FACTOR = 5;

    final double speed;
    switch (zone) {
      case CRUISE:
        {
          double delta = currentSpeed - maxSpeed;
          if (Math.abs(delta) < 1e-3) {
            if (DEBUG) System.out.printf("CRUISE: at max speed %.2f\n", maxSpeed);
            speed = maxSpeed;
          } else if (delta > 0) {
            // too fast! slow down ASAP but not discontinuously
            speed = Math.max(currentSpeed - (delta / FACTOR), maxSpeed); // todo: smooth this
            if (DEBUG)
              System.out.printf(
                  "CRUISE: too fast! slow down - current %.2f, max %.2f, delta %.2f, return %.2f\n",
                  currentSpeed, maxSpeed, delta, speed);
          } else {
            // too slow! speed up ASAP but not discontinuously
            speed = Math.min(currentSpeed + (-delta / FACTOR), maxSpeed); // todo: smooth this
            if (DEBUG)
              System.out.printf(
                  "CRUISE: too slow! speed up - current %.2f, max %.2f, delta %.2f, return %.2f\n",
                  currentSpeed, maxSpeed, delta, speed);
          }
          break;
        }
      case ACCEL:
        {
          double target =
              maxSpeedAccelZone(
                  requestedAcceleration, minSpeed, distanceToTravelInAccelZone, accelDistance);
          double delta = currentSpeed - target;
          if (Math.abs(delta) < 1e-3) {
            speed = target;
            if (DEBUG)
              System.out.printf(
                  "ACCEL: at target %.2f, left in zone: %.2f\n",
                  target, distanceToTravelInAccelZone);
          } else if (delta > 0) {
            // too fast! slow down ASAP but not discontinuously
            speed = Math.max(currentSpeed - (delta / FACTOR), target); // todo: smooth this
            if (DEBUG)
              System.out.printf(
                  "ACCEL: too fast! slow down - current %.2f, target %.2f, delta %.2f, return %.2f, left in zone: %.2f\n",
                  currentSpeed, target, delta, speed, distanceToTravelInAccelZone);

          } else {
            // too slow! speed up ASAP but not discontinuously
            speed = Math.min(currentSpeed + (-delta / FACTOR), target); // todo: smooth this
            if (DEBUG)
              System.out.printf(
                  "ACCEL: too slow! speed up - current %.2f, target %.2f, delta %.2f, return %.2f, left in zone: %.2f\n",
                  currentSpeed, target, delta, speed, distanceToTravelInAccelZone);
          }
          break;
        }
      case DECEL:
        {
          // define the curve
          double target =
              maxSpeedDecelZone(requestedDeceleration, maxSpeed, distanceToTravel, decelDistance);
          double delta = currentSpeed - target;
          if (Math.abs(delta) < 1e-3) {
            speed = target;
            if (DEBUG) System.out.printf("DECEL: at target %.2f\n", target);
          } else if (delta > 0) {
            // too fast! slow down ASAP but not discontinuously
            speed = Math.max(currentSpeed - (delta / FACTOR), target); // todo: smooth this
            if (DEBUG)
              System.out.printf(
                  "DECEL: too fast! slow down - current %.2f, target %.2f, delta %.2f,  return %.2f\n",
                  currentSpeed, target, delta, speed);
          } else {
            // too slow! speed up ASAP but not discontinuously
            speed = Math.min(currentSpeed + (-delta / FACTOR), target); // todo: smooth this
            if (DEBUG)
              System.out.printf(
                  "DECEL: too slow! speed up - current %.2f, target %.2f, delta %.2f, return %.2f\n",
                  currentSpeed, target, delta, speed);
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
      double deceleration,
      double maxSpeed,
      double distanceToTravelInDecelZone,
      double decelDistance) {
    if (distanceToTravelInDecelZone > decelDistance)
      throw new IllegalArgumentException(
          "Distance to travel in deceleration zone ("
              + String.format("%.2f", distanceToTravelInDecelZone)
              + ") cannot exceed the deceleration distance ("
              + String.format("%.2f", decelDistance)
              + ")");
    return (deceleration * distanceToTravelInDecelZone) + maxSpeed;
  }

  static double maxSpeedAccelZone(
      double acceleration,
      double minSpeed,
      double distanceToTravelInAccelZone,
      double accelDistance) {
    if (distanceToTravelInAccelZone > accelDistance)
      throw new IllegalArgumentException(
          "Distance to travel in acceleration zone ("
              + String.format("%.2f", distanceToTravelInAccelZone)
              + ") cannot exceed the acceleration distance ("
              + String.format("%.2f", accelDistance)
              + ")");
    return (acceleration * distanceToTravelInAccelZone) + minSpeed;
  }
}
