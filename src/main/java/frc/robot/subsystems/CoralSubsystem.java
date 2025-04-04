package frc.robot.subsystems;

import static frc.robot.Constants.CoralConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.ArmAngle;
import frc.robot.Constants.CoralConstants.ElevatorHeight;
import frc.robot.Robot;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.telemetry.Telemetry;

public class CoralSubsystem extends SubsystemBase {

  private class SensorCache {

    double elevatorEncoderSpeed = 0;
    double elevatorEncoderPosition = 0;
    double previousElevatorEncoderPosition = 0;
    double previousElevatorEncoderHeight = 0;

    double digitalElevatorEncoderPosition = 0;

    boolean elevatorUpperLimitReached = false;
    boolean elevatorLowerLimitReached = false;

    double armEncoderSpeed = 0;
    double armEncoderAngle = 0;

    double intakeEncoderSpeed = 0;
    double intakeEncoderPosition = 0;

    boolean coralDetected = false;
    boolean prevCoralDetected = false;
  }

  // Coral Subsystem Motors
  private final SparkFlex elevatorMotor =
      new SparkFlex(CoralConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkFlex armMotor =
      new SparkFlex(CoralConstants.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax intakeMotor =
      new SparkMax(CoralConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

  private final Alert elevatorMotorFault =
      new Alert("Elevator Motor Fault Detected!", Alert.AlertType.kError);
  private final Alert armMotorFault =
      new Alert("Arm Motor Fault Detected!", Alert.AlertType.kError);
  private final Alert intakeMotorFault =
      new Alert("Intake Motor Fault Detected!", Alert.AlertType.kError);

  private final Encoder digitalElevatorEncoder = new Encoder(0, 1);

  private double elevatorSetpoint = 0;
  private double elevatorSpeed = 0;
  private double armSetpoint = 0;
  private double armSpeed = 0;
  private double intakeSetpoint = 0;
  private double lastKnownElevatorHeight = -1;

  // Elevator

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private SparkLimitSwitch elevatorLowerLimitSwitch = elevatorMotor.getReverseLimitSwitch();
  private SparkLimitSwitch elevatorUpperLimitSwitch = elevatorMotor.getForwardLimitSwitch();

  private double elevatorEncoderOffset = 0;
  private double digitalElevatorEncoderOffset = 0;

  // Arm

  private RelativeEncoder armEncoder = armMotor.getEncoder();
  private SparkAbsoluteEncoder armAngleEncoder = armMotor.getAbsoluteEncoder();

  private double armEncoderOffset = 0;
  private boolean armAboveThreshold;

  // Intake

  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private SparkLimitSwitch intakeCoralDetector = intakeMotor.getForwardLimitSwitch();

  private SparkMaxConfig intakeSparkMaxConfig;

  // Sensor Cache
  private final SensorCache sensorCache = new SensorCache();

  // Simulation constants
  private boolean isSimulation = false;
  // Elevator full speed up: the elevator will raise 60 inches in 2 seconds with a loop time of
  // 20ms.
  private static final double ELEVATOR_MAX_UP_DISTANCE_PER_LOOP = 60 * .02 / 2;
  // Elevator full speed down: the elevator will lower in 1.5 seconds.
  private static final double ELEVATOR_MAX_DOWN_DISTANCE_PER_LOOP = 60 * .02 / 1.5;
  private double simulationElevatorHeight = 0;
  // Arm full speed: the arm will raise 180 degrees in two secondsconds.
  private static final double ARM_ANGLE_MAX_DEGREES_PER_LOOP = 180 * .02 / 2.0;
  private double simulationArmAngle = 0;
  // Intake detect time seconds.
  private static final double INTAKE_DETECTION_TIME_SECONDS = 3;
  private Timer simulationIntakeDetectTimer = new Timer();
  private boolean simulationIntakeDetector = false;
  private double simulationPreviousIntakeSpeed = 0;
  private int simulationIntakeEncoder = 0;

  public CoralSubsystem() {
    Telemetry.coral.enabled = Constants.TelemetryConfig.coral;

    /*
     * Elevator Motor Config
     */
    SparkFlexConfig flexConfig = new SparkFlexConfig();

    flexConfig.disableFollowerMode();
    flexConfig.idleMode(IdleMode.kBrake);
    flexConfig.inverted(CoralConstants.ELEVATOR_MOTOR_INVERTED);

    // Limit the current to 20A max
    // flexConfig.smartCurrentLimit(20);

    // Upper and Lower Limit switches
    flexConfig.limitSwitch.forwardLimitSwitchEnabled(false);
    flexConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    flexConfig.limitSwitch.reverseLimitSwitchEnabled(false);
    flexConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);

    elevatorMotor.configure(
        flexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder.setPosition(0);

    /*
     * Arm Motor Config
     */
    SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();

    sparkFlexConfig.disableFollowerMode();
    sparkFlexConfig.idleMode(IdleMode.kBrake);
    sparkFlexConfig.inverted(CoralConstants.ARM_MOTOR_INVERTED);

    // Limit the current to 20A max
    // flexConfig.smartCurrentLimit(20);

    sparkFlexConfig.absoluteEncoder.inverted(CoralConstants.ARM_ANGLE_ENCODER_INVERTED);
    sparkFlexConfig.absoluteEncoder.zeroOffset(CoralConstants.ARM_ANGLE_ENCODER_OFFSET);

    armMotor.configure(
        sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armAboveThreshold =
        armAngleEncoder.getPosition() > CoralConstants.ARM_CAMERA_THRESHOLD_POSITION;

    /*
     * Intake Motor Config
     */
    intakeSparkMaxConfig = new SparkMaxConfig();

    intakeSparkMaxConfig.disableFollowerMode();
    intakeSparkMaxConfig.idleMode(IdleMode.kBrake);
    intakeSparkMaxConfig.inverted(CoralConstants.INTAKE_MOTOR_INVERTED);

    // Limit the current to 20A max
    // flexConfig.smartCurrentLimit(20);

    intakeSparkMaxConfig.limitSwitch.forwardLimitSwitchEnabled(false);
    intakeSparkMaxConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    intakeMotor.configure(
        intakeSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /*
     * Simulation
     */
    if (Robot.isSimulation()) {
      isSimulation = true;
    }
  }

  /*
   * Sensor Cache
   *
   * A sensor cache is used to avoid reading sensors multiple times in a loop.
   * The sensors are read once at the beginning of each loop and cache can be used
   * by the subsystem and the co
   */
  public void updateSensorCache() {

    /*
     * Elevator
     */
    sensorCache.elevatorEncoderSpeed = elevatorEncoder.getVelocity();
    sensorCache.elevatorEncoderPosition = elevatorEncoder.getPosition();

    sensorCache.digitalElevatorEncoderPosition = digitalElevatorEncoder.getRaw();

    sensorCache.previousElevatorEncoderPosition = sensorCache.elevatorEncoderPosition;
    sensorCache.previousElevatorEncoderHeight = getElevatorEncoder();

    sensorCache.elevatorLowerLimitReached = elevatorLowerLimitSwitch.isPressed();
    sensorCache.elevatorUpperLimitReached = elevatorUpperLimitSwitch.isPressed();

    /*
     * Arm
     */
    sensorCache.armEncoderSpeed = armEncoder.getVelocity();
    sensorCache.armEncoderAngle = armAngleEncoder.getPosition();

    /*
     * Intake
     */
    sensorCache.intakeEncoderPosition = intakeEncoder.getPosition();
    sensorCache.intakeEncoderSpeed = intakeEncoder.getVelocity();

    sensorCache.prevCoralDetected = sensorCache.coralDetected;
    sensorCache.coralDetected = intakeCoralDetector.isPressed();
  }

  /*
   * Elevator Routines
   */
  public void setElevatorSpeed(double speed) {

    this.elevatorSetpoint = speed;
  }

  public boolean moveElevatorToHeight(ElevatorHeight targetHeight) {

    if (isAtElevatorHeight(targetHeight)) {
      setElevatorSpeed(0);
      return true;
    }
    double speed;

    double error = targetHeight.encoderCount - getElevatorEncoder();

    if (Math.abs(error) >= CoralConstants.ELEVATOR_SLOW_ZONE) {
      speed = CoralConstants.ELEVATOR_MAX_SPEED;
    } else {
      speed = CoralConstants.ELEVATOR_SLOW_ZONE_SPEED;
    }
    speed *= Math.signum(error);

    // if you deployed code with the elevator up, go down slowly
    if (targetHeight == ElevatorHeight.COMPACT && getElevatorEncoder() <= 0) {
      speed = -ELEVATOR_SLOW_ZONE_SPEED;
    }
    if (targetHeight == ElevatorHeight.LEVEL_4 && getElevatorEncoder() >= ELEVATOR_MAX_HEIGHT) {
      speed = ELEVATOR_SLOW_ZONE_SPEED;
    }

    setElevatorSpeed(speed);

    return false;
  }

  public boolean isAtElevatorHeight(ElevatorHeight height) {

    if (height == ElevatorHeight.COMPACT) {
      return isElevatorAtLowerLimit();
    }
    if (height == ElevatorHeight.LEVEL_4) {
      return isElevatorAtUpperLimit();
    }

    return (Math.abs(height.encoderCount - getElevatorEncoder())
        <= CoralConstants.ELEVATOR_TOLERANCE);
  }

  public double getThomasHeightM() {
    if (isElevatorAtLowerLimit()) {
      return CoralConstants.THOMAS_STARTING_HEIGHT;
    }

    double encoderCount = getElevatorEncoder();
    return CoralConstants.ELEVATOR_METERS_PER_ENCODER_COUNT * encoderCount
        + CoralConstants.THOMAS_STARTING_HEIGHT;
  }

  public boolean isElevatorAtLowerLimit() {

    if (isSimulation) {
      if (simulationElevatorHeight <= 0) {
        return true;
      } else {
        return false;
      }
    }
    return sensorCache.elevatorLowerLimitReached;
  }

  public boolean isElevatorAtUpperLimit() {

    if (isSimulation) {
      if (simulationElevatorHeight >= 60) {
        return true;
      } else {
        return false;
      }
    }
    return sensorCache.elevatorUpperLimitReached;
  }

  public double getElevatorEncoder() {

    if (isSimulation) {
      return simulationElevatorHeight + elevatorEncoderOffset;
    }
    return getDigitalElevatorEncoder() * 163.05 / 116532;
    // return sensorCache.elevatorEncoderPosition + elevatorEncoderOffset;
  }

  public double getDigitalElevatorEncoder() {
    return sensorCache.digitalElevatorEncoderPosition + digitalElevatorEncoderOffset;
  }

  public void resetElevatorEncoder() {
    // log a message if we reset the encoder if we are off by more than 4000 digital encoder counts.
    // (the lower limit switch is active for a range of about 3650 counts).
    if (sensorCache.digitalElevatorEncoderPosition > 4000) {
      System.out.println(
          "************ UNEXPECTED ELEVATOR ENCODER RESET!!! OLD VALUE: "
              + sensorCache.digitalElevatorEncoderPosition
              + " ************");
    }
    setElevatorEncoder(0);
    setDigitalElevatorEncoder(0);
    // System.out.println("Resetting elevator encoder!");
  }

  public void setElevatorEncoder(double encoderValue) {

    elevatorEncoderOffset = 0;
    elevatorEncoderOffset = -getElevatorEncoder() + encoderValue;

    // Reset the previous value in the sensor cache
    sensorCache.previousElevatorEncoderHeight = encoderValue;
    setDigitalElevatorEncoder(encoderValue / (163.05 / 116532));
  }

  public void setDigitalElevatorEncoder(double digitalEncoderValue) {
    digitalElevatorEncoderOffset = 0;
    digitalElevatorEncoderOffset = -getDigitalElevatorEncoder() + digitalEncoderValue;
  }

  /*
   * Arm Routines
   */
  public void setArmSpeed(double speed) {
    armSetpoint = speed;
  }

  public boolean isArmAtLowerLimit() {

    return getArmAngle() <= CoralConstants.ARM_LOWER_LIMIT_POSITION;
  }

  public boolean isArmAtUpperLimit() {

    return getArmAngle() >= CoralConstants.ARM_UPPER_LIMIT_POSITION;
  }

  public double getArmAngle() {

    if (isSimulation) {
      return simulationArmAngle + armEncoderOffset;
    }

    return (sensorCache.armEncoderAngle - 0.1) * 360;
  }

  public void resetArmEncoder() {
    setArmEncoderPostion(0);
  }

  public void setArmEncoderPostion(double encoderValue) {
    armEncoderOffset = 0;
    armEncoderOffset = -getArmAngle() + encoderValue;
  }

  public boolean moveArmToAngle(ArmAngle armAngle) {

    double currentAngle = getArmAngle();

    double angleError = armAngle.angle - currentAngle;
    double desiredArmSpeed = CoralConstants.ARM_FAST_SPEED;

    if (Math.abs(angleError) < CoralConstants.ARM_ANGLE_TOLERANCE) {
      armSetpoint = 0;
      setArmSpeed(0);
      return true;
    }

    if (Math.abs(angleError) < CoralConstants.ARM_SLOW_ZONE_ANGLE) {
      desiredArmSpeed = CoralConstants.ARM_SLOW_ZONE_SPEED;
    }

    if (angleError < 0) {
      desiredArmSpeed = -desiredArmSpeed;
    }

    armSetpoint = desiredArmSpeed;
    setArmSpeed(desiredArmSpeed);
    return false;
  }

  /*
   * Intake Routines
   */
  public void setIntakeSpeed(double speed) {

    this.intakeSetpoint = speed;
    intakeMotor.set(this.intakeSetpoint);
  }

  public boolean isCoralDetected() {

    if (isSimulation) {
      return simulationIntakeDetector;
    }
    return sensorCache.coralDetected;
  }

  public double getIntakeEncoder() {

    if (isSimulation) {
      return simulationIntakeEncoder;
    }

    return sensorCache.intakeEncoderPosition;
  }

  public void setIntakeHardLimit(boolean limitEnabled) {

    intakeSparkMaxConfig.limitSwitch.forwardLimitSwitchEnabled(limitEnabled);

    intakeMotor.configure(
        intakeSparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void stop() {
    setElevatorSpeed(0);
    setArmSpeed(0);
    setIntakeSpeed(0);
  }

  /*
   * Periodic routines
   */
  @Override
  public void periodic() {

    updateSensorCache();

    if (!sensorCache.prevCoralDetected && sensorCache.coralDetected) {
      OperatorInput.setRumblePattern(OperatorInput.RumblePattern.SHORT);
    }

    if (isSimulation) {
      simulate();
    }

    checkSafety();

    LightingSubsystem.coralInIntake = isCoralDetected();
    LightingSubsystem.isIntaking = intakeEncoder.getVelocity() > 0.1 || intakeEncoder.getVelocity() < -0.1;

    Telemetry.coral.digitalElevatorPosition = getDigitalElevatorEncoder();
    Telemetry.coral.rawDigitalElevatorEncoder = digitalElevatorEncoder.getRaw();
    Telemetry.coral.digitalElevatorEncoderOffset = digitalElevatorEncoderOffset;
    Telemetry.coral.elevatorPosition = getElevatorEncoder();
    Telemetry.coral.elevatorUpperLimit = isElevatorAtUpperLimit();
    Telemetry.coral.elevatorLowerLimit = isElevatorAtLowerLimit();
    Telemetry.coral.armAngle = getArmAngle();
    Telemetry.coral.coralDetected = isCoralDetected();

    if (Telemetry.coral.enabled) {
      Telemetry.coral.elevatorSetpoint = elevatorSetpoint;
      Telemetry.coral.elevatorSpeed = elevatorSpeed;

      Telemetry.coral.armSpeed = armSetpoint;
      Telemetry.coral.armUpperLimit = isArmAtUpperLimit();
      Telemetry.coral.armLowerLimit = isArmAtLowerLimit();

      Telemetry.coral.intakeSpeed = intakeSetpoint;
    }

    elevatorMotorFault.set(elevatorMotor.hasActiveFault());
    armMotorFault.set(armMotor.hasActiveFault());
    intakeMotorFault.set(intakeMotor.hasActiveFault());
  }

  private void simulate() {

    // This loop will be called every 20 ms, 50 times per second

    // Move the elevator up or down depending on the direction of the motor speed
    // The elevator will fall faster than it will lift.
    if (elevatorSpeed > 0) {
      simulationElevatorHeight += ELEVATOR_MAX_UP_DISTANCE_PER_LOOP * elevatorSpeed;
    }
    if (elevatorSpeed < 0) {
      simulationElevatorHeight += ELEVATOR_MAX_DOWN_DISTANCE_PER_LOOP * elevatorSpeed;
    }

    simulationArmAngle += ARM_ANGLE_MAX_DEGREES_PER_LOOP * armSetpoint;

    simulationIntakeEncoder += intakeSetpoint;

    // Intake detection, change states if the timer is running for 3 seconds
    if (intakeSetpoint != 0 && simulationPreviousIntakeSpeed == 0) {
      simulationIntakeDetectTimer.reset();
      simulationIntakeDetectTimer.start();
    }
    simulationPreviousIntakeSpeed = intakeSetpoint;

    if (intakeSetpoint != 0) {
      if (simulationIntakeDetectTimer.hasElapsed(INTAKE_DETECTION_TIME_SECONDS)) {
        simulationIntakeDetector = !simulationIntakeDetector;
        simulationIntakeDetectTimer.reset();
        simulationIntakeDetectTimer.stop();
      }
    } else {
      simulationIntakeDetectTimer.reset();
      simulationIntakeDetectTimer.stop();
    }
  }

  private void checkSafety() {

    boolean elLowerLimit = isElevatorAtLowerLimit();
    boolean elUpperLimit = isElevatorAtUpperLimit();
    boolean elGoingDown = elevatorSetpoint < 0;
    boolean elGoingUp = elevatorSetpoint > 0;

    if (elLowerLimit) {
      resetElevatorEncoder();

      if (elGoingDown) {
        elevatorSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        elevatorMotor.set(0);
      }
    }

    if (elUpperLimit) {
      // setElevatorEncoder(ELEVATOR_MAX_HEIGHT);
      //      elevatorEncoder.setPosition(ELEVATOR_MAX_HEIGHT);

      if (elGoingUp) {
        elevatorSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        elevatorMotor.set(0);
      }
    }

    if (!(elLowerLimit && elGoingDown) && !(elUpperLimit && elGoingUp)) {

      // If not at either limit, then limit the speed.
      double previousMotorSpeed = elevatorSpeed;

      this.elevatorSpeed = this.elevatorSetpoint;

      // Elevator is in the lower slow zone
      if (getElevatorEncoder() <= CoralConstants.ELEVATOR_SLOW_ZONE) {
        // System.out.println("Lower slow zone!");

        if (elevatorSpeed < -CoralConstants.ELEVATOR_SLOW_ZONE_SPEED) {
          elevatorSpeed = -CoralConstants.ELEVATOR_SLOW_ZONE_SPEED;
        }
      }
      // Elevator is in the upper slow zone
      else if (getElevatorEncoder()
          >= CoralConstants.ELEVATOR_MAX_HEIGHT - CoralConstants.ELEVATOR_SLOW_ZONE) {
        // System.out.println("Upper slow zone!");

        if (elevatorSpeed > CoralConstants.ELEVATOR_SLOW_ZONE_SPEED) {
          elevatorSpeed = CoralConstants.ELEVATOR_SLOW_ZONE_SPEED;
        }
      } else { // Elevator is not near a limit

        // Limit the elevator speed
        if (Math.abs(elevatorSpeed) > CoralConstants.ELEVATOR_MAX_SPEED) {
          elevatorSpeed = CoralConstants.ELEVATOR_MAX_SPEED * Math.signum(elevatorSpeed);
        }
      }

      // Rate limit to ELEVATOR_MAX_SLEW speed change per cycle
      double delta = (elevatorSpeed - previousMotorSpeed);
      if (Math.abs(delta) > ELEVATOR_MAX_SLEW) {
        elevatorSpeed = previousMotorSpeed + Math.signum(delta) * ELEVATOR_MAX_SLEW;
      }

      // Directly set the motor speed, do not call the setter method (recursive loop)
      elevatorMotor.set(elevatorSpeed);
    }

    /*
     * Arm Safety
     */
    boolean armLowerLimit = isArmAtLowerLimit();
    boolean armUpperLimit = isArmAtUpperLimit();
    boolean armGoingDown = armSetpoint < 0;
    boolean armGoingUp = armSetpoint > 0;

    if (armLowerLimit) {

      if (armSetpoint < 0) {
        armSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        armMotor.set(0);
      }
    }

    if (armUpperLimit) {

      if (armSetpoint > 0) {
        armSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        armMotor.set(0);
      }
    }

    // If not at either limit, then limit the arm speed
    if (!(armLowerLimit && armGoingDown) && !(armUpperLimit && armGoingUp)) {

      double previousArmMotorSpeed = armSpeed;
      this.armSpeed = this.armSetpoint;

      // If near the lower limit, then limit the speed
      if (getArmAngle()
          < CoralConstants.ARM_LOWER_LIMIT_POSITION + CoralConstants.ARM_SLOW_ZONE_ANGLE) {

        if (armSpeed < -CoralConstants.ARM_SLOW_ZONE_SPEED) {

          armSpeed = -CoralConstants.ARM_SLOW_ZONE_SPEED;
        }
      }
      // If near the upper limit, limit the speed
      else if (getArmAngle()
          > CoralConstants.ARM_UPPER_LIMIT_POSITION - CoralConstants.ARM_SLOW_ZONE_ANGLE) {

        if (armSpeed > CoralConstants.ARM_SLOW_ZONE_SPEED) {

          armSpeed = CoralConstants.ARM_SLOW_ZONE_SPEED;
        }
      } else {

        // Limit the elevator speed
        if (Math.abs(armSpeed) > CoralConstants.ARM_MAX_SPEED) {
          armSpeed = CoralConstants.ARM_MAX_SPEED * Math.signum(armSpeed);
          // Directly set the motor speed, do not call the setter method (recursive loop)
        }
      }

      // Rate limit to ELEVATOR_MAX_SLEW speed change per cycle
      double armDelta = (armSpeed - previousArmMotorSpeed);
      if (Math.abs(armDelta) > ARM_MAX_SLEW) {
        armSpeed = previousArmMotorSpeed + Math.signum(armDelta) * ARM_MAX_SLEW;
      }

      armMotor.set(armSpeed);
    }

    /*
     * INTAKE SAFETY
     */

    intakeMotor.set(intakeSetpoint);
  }

  @Override
  public String toString() {

    StringBuilder sb = new StringBuilder();

    sb.append(this.getClass().getSimpleName())
        .append(" : ")
        .append("Elevator: speed ")
        .append(elevatorSpeed)
        .append(" height ")
        .append(getElevatorEncoder())
        .append("in")
        .append(",  Arm: speed ")
        .append(armSetpoint)
        .append(" angle ")
        .append(getArmAngle())
        .append(" deg")
        .append(",  Intake: speed ")
        .append(intakeSetpoint)
        .append(" coral detect: ")
        .append(isCoralDetected());

    return sb.toString();
  }
}
