package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.ArmAngle;
import frc.robot.Constants.CoralConstants.ElevatorHeight;
import frc.robot.Robot;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

import static frc.robot.Constants.CoralConstants.*;

public class CoralSubsystem extends SubsystemBase {

  private class SensorCache {

    double elevatorEncoderSpeed = 0;
    double elevatorEncoderPosition = 0;

    boolean elevatorUpperLimitReached = false;
    boolean elevatorLowerLimitReached = false;

    double armEncoderSpeed = 0;
    double armEncoderAngle = 0;

    double intakeEncoderSpeed = 0;
    double intakeEncoderPosition = 0;

    boolean coralDetected = false;
  }

  // Coral Subsystem Motors
  private final SparkFlex elevatorMotor =
      new SparkFlex(CoralConstants.ELEVATOR_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax armMotor =
      new SparkMax(CoralConstants.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
  private final SparkMax intakeMotor =
      new SparkMax(CoralConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

  private double elevatorSpeed = 0;
  private double armSpeed = 0;
  private double intakeSpeed = 0;
// ultrasonic
  private final AnalogInput ultrasonicDistanceSensor = new AnalogInput(ULTRASONIC_SENSOR_PORT);


  // Elevator

  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private SparkLimitSwitch elevatorLowerLimitSwitch = elevatorMotor.getReverseLimitSwitch();
  private SparkLimitSwitch elevatorUpperLimitSwitch = elevatorMotor.getForwardLimitSwitch();

  private double elevatorEncoderOffset = 0;

  // Arm

  private RelativeEncoder armEncoder = armMotor.getEncoder();
  private SparkAbsoluteEncoder armAngleEncoder = armMotor.getAbsoluteEncoder();

  private double armEncoderOffset = 0;
  private boolean armAboveThreshold;

  // Intake

  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private SparkLimitSwitch intakeCoralDetector = intakeMotor.getForwardLimitSwitch();

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

  private LimelightVisionSubsystem visionSubsystem;

  public CoralSubsystem(LimelightVisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;

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
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    sparkMaxConfig.disableFollowerMode();
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(CoralConstants.ARM_MOTOR_INVERTED);

    // Limit the current to 20A max
    // flexConfig.smartCurrentLimit(20);

    sparkMaxConfig.absoluteEncoder.inverted(CoralConstants.ARM_ANGLE_ENCODER_INVERTED);
    sparkMaxConfig.absoluteEncoder.zeroOffset(CoralConstants.ARM_ANGLE_ENCODER_OFFSET);

    armMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armAboveThreshold =
        armAngleEncoder.getPosition() > CoralConstants.ARM_CAMERA_THRESHOLD_POSITION;
    setCamStream();

    /*
     * Intake Motor Config
     */
    sparkMaxConfig = new SparkMaxConfig();

    sparkMaxConfig.disableFollowerMode();
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(CoralConstants.INTAKE_MOTOR_INVERTED);

    // Limit the current to 20A max
    // flexConfig.smartCurrentLimit(20);

    sparkMaxConfig.limitSwitch.forwardLimitSwitchEnabled(false);
    sparkMaxConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);

    intakeMotor.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    sensorCache.elevatorEncoderPosition = elevatorEncoder.getPosition() + elevatorEncoderOffset;

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
    sensorCache.coralDetected = intakeCoralDetector.isPressed();
  }

  private void updateVision() {
    boolean newAboveThreshold = getArmAngle() > CoralConstants.ARM_CAMERA_THRESHOLD_POSITION;
    if (newAboveThreshold != armAboveThreshold) {
      armAboveThreshold = newAboveThreshold;
      setCamStream();
    }
    visionSubsystem.setThomasHeight(getThomasHeightCM());
  }

  /*
   * Elevator Routines
   */
  public void setElevatorSpeed(double speed) {

    this.elevatorSpeed = speed;

    checkSafety();

    elevatorMotor.set(elevatorSpeed);
  }

  public boolean moveElevatorToHeight(ElevatorHeight targetHeight) {

    if (isAtElevatorHeight(targetHeight)) {
      setElevatorSpeed(0);
      return true;
    }
    double speed = 0;

    double error = targetHeight.encoderCount - getElevatorEncoder();

    if (Math.abs(error) >= 10) {
      speed = CoralConstants.ELEVATOR_MAX_SPEED;
    } else {
      speed = CoralConstants.ELEVATOR_SLOW_ZONE_SPEED;
    }
    speed *=Math.signum(error);

    // if you deployed code with the elevator up, go down slowly
    if (targetHeight == ElevatorHeight.COMPACT && getElevatorEncoder() <=0) {
      speed = -ELEVATOR_SLOW_ZONE_SPEED;
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

  public double getThomasHeightCM() {
    if (isElevatorAtLowerLimit())  {
      return CoralConstants.THOMAS_STARTING_HEIGHT;
    }

    double  encoderCount = getElevatorEncoder();
    return CoralConstants.ELEVATOR_CENTIMETERS_PER_ENCODER_COUNT * encoderCount + CoralConstants.THOMAS_STARTING_HEIGHT;
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
    return sensorCache.elevatorEncoderPosition;
  }

  public void resetElevatorEncoder() {
    setElevatorEncoder(0);
    System.out.println("Resetting elevator encoder!");
  }

  public void setElevatorEncoder(double encoderValue) {

    elevatorEncoderOffset = 0;
    elevatorEncoderOffset = -getElevatorEncoder() + encoderValue;
  }

  /*
   * Arm Routines
   */
  public void setArmSpeed(double speed) {
    armSpeed = speed;

    checkSafety();

    armMotor.set(armSpeed);
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
      armSpeed = 0;
      setArmSpeed(0);
      return true;
    }

    if (Math.abs(angleError) < CoralConstants.ARM_SLOW_ZONE_ANGLE) {
      desiredArmSpeed = CoralConstants.ARM_SLOW_ZONE_SPEED;
    }

    if (angleError < 0) {
      desiredArmSpeed = -desiredArmSpeed;
    }

    armSpeed = desiredArmSpeed;
    setArmSpeed(desiredArmSpeed);
    return false;
  }

  /*
   * Intake Routines
   */
  public void setIntakeSpeed(double speed) {

    this.intakeSpeed = speed;

    checkSafety();

    intakeMotor.set(intakeSpeed);
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

  public double getUltrasonicDistanceCm() {
    double ultrasonicVoltage = ultrasonicDistanceSensor.getVoltage();
    double distanceCm = ULTRASONIC_M * ultrasonicVoltage + ULTRASONIC_B;
    return Math.round(distanceCm);
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
    updateVision();

    if (isSimulation) {
      simulate();
    }



    checkSafety();

    SmartDashboard.putNumber("Coral/Elevator Speed", elevatorSpeed);
    SmartDashboard.putNumber("Coral/Elevator Position", getElevatorEncoder());
    SmartDashboard.putBoolean("Coral/Elevator Upper Limit", isElevatorAtUpperLimit());
    SmartDashboard.putBoolean("Coral/Elevator Lower Limit", isElevatorAtLowerLimit());

    SmartDashboard.putNumber("Coral/Arm Speed", armSpeed);
    SmartDashboard.putNumber("Coral/Arm Angle", getArmAngle());
    SmartDashboard.putBoolean("Coral/Arm Upper Limit", isArmAtUpperLimit());
    SmartDashboard.putBoolean("Coral/Arm Lower Limit", isArmAtLowerLimit());

    SmartDashboard.putNumber("Coral/Intake Speed", intakeSpeed);
    SmartDashboard.putBoolean("Coral/Coral Detected", isCoralDetected());

    SmartDashboard.putNumber("Coral/Ultrasonic Distance Cm", getUltrasonicDistanceCm());
    SmartDashboard.putNumber("Coral/Ultrasonic Voltage", ultrasonicDistanceSensor.getVoltage());
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

    simulationArmAngle += ARM_ANGLE_MAX_DEGREES_PER_LOOP * armSpeed;

    simulationIntakeEncoder += intakeSpeed;

    // Intake detection, change states if the timer is running for 3 seconds
    if (intakeSpeed != 0 && simulationPreviousIntakeSpeed == 0) {
      simulationIntakeDetectTimer.reset();
      simulationIntakeDetectTimer.start();
    }
    simulationPreviousIntakeSpeed = intakeSpeed;

    if (intakeSpeed != 0) {
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

    if (isElevatorAtLowerLimit()) {

      if (elevatorSpeed < 0) {
        elevatorSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        elevatorMotor.set(0);
        resetElevatorEncoder();
      }
    }

    if (isElevatorAtUpperLimit()) {

      if (elevatorSpeed > 0) {
        elevatorSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        elevatorMotor.set(0);
      }
    }

    // If not at either limit, then limit the speed.
    if (!isElevatorAtLowerLimit() && !isElevatorAtUpperLimit()) {

      // Elevator is in the lower slow zone
      if (getElevatorEncoder() <= CoralConstants.ELEVATOR_SLOW_ZONE) {

        if (elevatorSpeed < -CoralConstants.ELEVATOR_SLOW_ZONE_SPEED) {

          elevatorSpeed = -CoralConstants.ELEVATOR_SLOW_ZONE_SPEED;
          // Directly set the motor speed, do not call the setter method (recursive loop)
          elevatorMotor.set(elevatorSpeed);
        }
      }
      // Elevator is in the upper slow zone
      else if (getElevatorEncoder()
          >= CoralConstants.ELEVATOR_MAX_HEIGHT - CoralConstants.ELEVATOR_SLOW_ZONE) {

        if (elevatorSpeed > CoralConstants.ELEVATOR_SLOW_ZONE_SPEED) {

          elevatorSpeed = CoralConstants.ELEVATOR_SLOW_ZONE_SPEED;
          // Directly set the motor speed, do not call the setter method (recursive loop)
          elevatorMotor.set(elevatorSpeed);
        }
      } else { // Elevator is not near a limit

        // Limit the elevator speed
        if (Math.abs(elevatorSpeed) > CoralConstants.ELEVATOR_MAX_SPEED) {

          elevatorSpeed = CoralConstants.ELEVATOR_MAX_SPEED * Math.signum(elevatorSpeed);
          // Directly set the motor speed, do not call the setter method (recursive loop)
          elevatorMotor.set(elevatorSpeed);
        }
      }
    }

    /*
     * Arm Safety
     */
    if (isArmAtLowerLimit()) {

      if (armSpeed < 0) {
        armSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        armMotor.set(0);
      }
    }

    if (isArmAtUpperLimit()) {

      if (armSpeed > 0) {
        armSpeed = 0;
        // Directly set the motor speed, do not call the setter method (recursive loop)
        armMotor.set(0);
      }
    }

    // If not at either limit, then limit the arm speed
    if (!isArmAtLowerLimit() && !isArmAtUpperLimit()) {

      // If near the lower limit, then limit the speed
      if (getArmAngle()
          < CoralConstants.ARM_LOWER_LIMIT_POSITION + CoralConstants.ARM_SLOW_ZONE_ANGLE) {

        if (armSpeed < -CoralConstants.ARM_SLOW_ZONE_SPEED) {

          armSpeed = -CoralConstants.ARM_SLOW_ZONE_SPEED;
          armMotor.set(armSpeed);
        }
      }
      // If near the upper limit, limit the speed
      else if (getArmAngle()
          > CoralConstants.ARM_UPPER_LIMIT_POSITION - CoralConstants.ARM_SLOW_ZONE_ANGLE) {

        if (armSpeed > CoralConstants.ARM_SLOW_ZONE_SPEED) {

          armSpeed = CoralConstants.ARM_SLOW_ZONE_SPEED;
          armMotor.set(armSpeed);
        }
      } else {

        // Limit the elevator speed
        if (Math.abs(armSpeed) > CoralConstants.ARM_MAX_SPEED) {
          armSpeed = CoralConstants.ARM_MAX_SPEED * Math.signum(armSpeed);
          // Directly set the motor speed, do not call the setter method (recursive loop)
          armMotor.set(armSpeed);
        }
      }
    }
  }

  private void setCamStream() {
    visionSubsystem.setCameraView(
        armAboveThreshold
            ? LimelightVisionSubsystem.CamStreamType.WEBCAM
            : LimelightVisionSubsystem.CamStreamType.LIMELIGHT);
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
        .append(armSpeed)
        .append(" angle ")
        .append(getArmAngle())
        .append(" deg")
        .append(",  Intake: speed ")
        .append(intakeSpeed)
        .append(" coral detect: ")
        .append(isCoralDetected());

    return sb.toString();
  }
}
