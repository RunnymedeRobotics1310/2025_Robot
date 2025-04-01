package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

  public static final String PREFIX = "1310/";

  public static Test test = new Test();
  public static DriveTelemetry drive = new DriveTelemetry();
  public static VisionTelemetry vision = new VisionTelemetry();
  public static CoralTelemetry coral = new CoralTelemetry();
  public static ClimbTelemetry climb = new ClimbTelemetry();

  public static boolean healthyRobot = false;

  private Telemetry() {}

  public static void post() {
    test.post();
    drive.post();
    vision.post();
    coral.post();
    climb.post();

    SmartDashboard.putBoolean(PREFIX + "RobotHealth", healthyRobot);
  }
}
