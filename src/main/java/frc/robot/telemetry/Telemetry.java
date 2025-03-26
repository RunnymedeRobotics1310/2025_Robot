package frc.robot.telemetry;

public class Telemetry {

  public static final String PREFIX = "1310/";

  public static Test test = new Test();
  public static DriveTelemetry drive = new DriveTelemetry();
  public static VisionTelemetry vision = new VisionTelemetry();
  public static CoralTelemetry coral = new CoralTelemetry();
  public static ClimbTelemetry climb = new ClimbTelemetry();

  private Telemetry() {}

  public static void post() {
    test.post();
    drive.post();
    vision.post();
    coral.post();
    climb.post();
  }
}
