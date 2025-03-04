package frc.robot;

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
}
