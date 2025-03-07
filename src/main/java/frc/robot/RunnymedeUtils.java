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
}
