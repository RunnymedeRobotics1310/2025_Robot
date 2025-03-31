package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.telemetry.Telemetry;

public class ClimbSubsystem extends SubsystemBase {

  private DoubleSolenoid climbPiston =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          ClimbConstants.CLIMB_FORWARDS_PNEUMATIC_PORT,
          ClimbConstants.CLIMB_REVERSE_PNEUMATIC_PORT);

  private DigitalInput climbSensor = new DigitalInput(ClimbConstants.CLIMB_SENSOR_DIO_PORT);

  public ClimbSubsystem() {
    Telemetry.climb.enabled = Constants.TelemetryConfig.climb;
    setClimbDeployed(false);
  }

  /**
   * @param deployClimb {@code true} to deploy the climber (climb), {@code false} to retract the
   *     climber (lower the robot)
   */
  public void setClimbDeployed(boolean deployClimb) {
    if (deployClimb) {
      climbPiston.set(DoubleSolenoid.Value.kForward);
    } else {
      climbPiston.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public boolean isClimbDeployed() {
    return climbPiston.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean isCageInPosition() {
    return !climbSensor.get();
  }

  /*
   * Periodic routines
   */
  @Override
  public void periodic() {

    checkSafety();

    boolean prevClimbDetected = Telemetry.climb.cageDetected;
    Telemetry.climb.cageDetected = isCageInPosition();

    // Only buzz when it changes
    if (Telemetry.climb.cageDetected && !prevClimbDetected) {
      OperatorInput.setRumblePattern(OperatorInput.RumblePattern.BLIP);
    }

    if (Telemetry.climb.enabled) {
      Telemetry.climb.climbDeployed = isClimbDeployed();
      Telemetry.climb.canDeploy = true;
    }
  }

  private void checkSafety() {

    // Are there any safety checks?
  }

  @Override
  public String toString() {

    StringBuilder sb = new StringBuilder();

    sb.append(this.getClass().getSimpleName())
        .append(" : ")
        .append("climb: ")
        .append(isClimbDeployed());

    return sb.toString();
  }
}
