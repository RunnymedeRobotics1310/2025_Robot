package frc.robot;

import java.util.Objects;

import edu.wpi.first.wpilibj.DriverStation;

public class RunnymedeUtils {

  private static DriverStation.Alliance alliance = null;

  public static DriverStation.Alliance getRunnymedeAlliance() {
    if (alliance == null) {
      DriverStation.getAlliance().ifPresent(value -> alliance = value);
    }

    return Objects.requireNonNullElse(alliance, DriverStation.Alliance.Red);
  }
}
