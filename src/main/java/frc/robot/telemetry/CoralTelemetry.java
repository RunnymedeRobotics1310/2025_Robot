package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * @author JZ
 * @since 2025-02-16 10:45
 */
public class CoralTelemetry {

  public boolean enabled = false;

  public double digitalElevatorPosition = -1;
  public double rawDigitalElevatorEncoder = -1;
  public double digitalElevatorEncoderOffset = -1;
  public double elevatorPosition = -1;
  public boolean elevatorUpperLimit = false;
  public boolean elevatorLowerLimit = false;
  public double armAngle = -1;
  public boolean coralDetected = false;

  public double elevatorSetpoint = -1;
  public double elevatorSpeed = -1;
  public double armSpeed = -1;
  public boolean armUpperLimit = false;
  public boolean armLowerLimit = false;
  public double intakeSpeed = -1;

  void post() {
    SmartDashboard.putNumber("Coral/Digital Elevator Position", digitalElevatorPosition);
    SmartDashboard.putNumber("Coral/Raw Digital Elevator Encoder", rawDigitalElevatorEncoder);
    SmartDashboard.putNumber("Coral/Digital Elevator Encoder Offset", digitalElevatorEncoderOffset);
    SmartDashboard.putNumber("Coral/Elevator Position", elevatorPosition);
    SmartDashboard.putBoolean("Coral/Elevator Upper Limit", elevatorUpperLimit);
    SmartDashboard.putBoolean("Coral/Elevator Lower Limit", elevatorLowerLimit);
    SmartDashboard.putNumber("Coral/Arm Angle", armAngle);
    SmartDashboard.putBoolean("Coral/Coral Detected", coralDetected);

    if (enabled) {
      SmartDashboard.putNumber("Coral/Elevator Setpoint", elevatorSetpoint);
      SmartDashboard.putNumber("Coral/Elevator Speed", elevatorSpeed);

      SmartDashboard.putNumber("Coral/Arm Speed", armSpeed);
      SmartDashboard.putBoolean("Coral/Arm Upper Limit", armUpperLimit);
      SmartDashboard.putBoolean("Coral/Arm Lower Limit", armLowerLimit);

      SmartDashboard.putNumber("Coral/Intake Speed", intakeSpeed);
    }
  }
}
