package frc.robot.telemetry;

public class Telemetry {

    public static final String PREFIX = "1310/";

    public static Test test = new Test();
    public static DriveTelemetry drive = new DriveTelemetry();

    private Telemetry() {}

    public static void post() {
        test.post();
        drive.post();
    }
}
