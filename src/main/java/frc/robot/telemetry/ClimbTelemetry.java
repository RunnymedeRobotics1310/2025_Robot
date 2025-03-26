package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author JZ
 * @since 2025-02-16 10:45
 */
public class ClimbTelemetry {

  public boolean enabled = false;

  public boolean cageDetected = false;
  public boolean climbDeployed = false;
  public boolean canDeploy = false;

  void post() {

    SmartDashboard.putBoolean("1310/Climb/CageSensor", cageDetected);

    if (enabled) {
      SmartDashboard.putBoolean("1310/Climb/Deployed", climbDeployed);
      SmartDashboard.putBoolean("1310/Climb/CanDeploy", canDeploy);
    }
  }
}
