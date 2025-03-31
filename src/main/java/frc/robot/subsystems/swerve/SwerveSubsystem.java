package frc.robot.subsystems.swerve;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.VisionConstants.VISION_PRIMARY_LIMELIGHT_NAME;

import ca.team1310.swerve.RunnymedeSwerveDrive;
import ca.team1310.swerve.core.SwerveMath;
import ca.team1310.swerve.utils.SwerveUtils;
import ca.team1310.swerve.vision.LimelightAwareSwerveDrive;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RunnymedeUtils;
import frc.robot.telemetry.Telemetry;

public class SwerveSubsystem extends SubsystemBase {

  private final RunnymedeSwerveDrive drive;
  private final SwerveDriveSubsystemConfig config;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter omegaLimiter;
  private final PIDController headingPIDController;
  private final AnalogInput ultrasonicDistanceSensor = new AnalogInput(ULTRASONIC_SENSOR_PORT);

  private double ultrasonicVoltage;
  private double ultrasonicDistanceM;

  private final Alert ultrasonicDisconnected =
      new Alert("Ultrasonic Sensor Disconnected!", Alert.AlertType.kError);

  public SwerveSubsystem(SwerveDriveSubsystemConfig config) {
    this.drive =
        new LimelightAwareSwerveDrive(
            config.coreConfig(),
            VISION_PRIMARY_LIMELIGHT_NAME,
            FIELD_EXTENT_METRES_X,
            FIELD_EXTENT_METRES_Y);
    this.config = config;
    this.xLimiter = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
    this.yLimiter = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
    this.omegaLimiter = new SlewRateLimiter(config.rotationConfig().maxAccelerationRadPS2());
    headingPIDController =
        new PIDController(
            config.rotationConfig().headingP(),
            config.rotationConfig().headingI(),
            config.rotationConfig().headingD());
    headingPIDController.enableContinuousInput(-180, 180);
    headingPIDController.setTolerance(2);
    Telemetry.drive.enabled = config.telemetryEnabled();

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig pathPlannerConfig = null;
    try {
      pathPlannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(
                TRANSLATION_CONFIG.velocityP(),
                TRANSLATION_CONFIG.velocityI(),
                TRANSLATION_CONFIG.velocityD()), // Translation PID constants
            new PIDConstants(
                ROTATION_CONFIG.headingP(),
                ROTATION_CONFIG.headingI(),
                ROTATION_CONFIG.headingD()) // Rotation PID constants
            ),
        pathPlannerConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          return RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public void periodic() {
    ultrasonicVoltage = ultrasonicDistanceSensor.getVoltage();
    ultrasonicDistanceM = 1.29338 * ultrasonicVoltage - 0.51803;

    ultrasonicDisconnected.set(ultrasonicVoltage <= 0.3);

    Telemetry.drive.ultrasonicDistanceM = Math.round(ultrasonicDistanceM * 1000d) / 1000d;
    Telemetry.drive.ultrasonicVoltage = ultrasonicVoltage;
  }

  /*
   * *********************************************************************************************
   * Core methods for controlling the drivebase
   */

  /**
   * Add limiters to the change in drive values. Note this may not scale evenly - one may reach
   * desired speed before another.
   *
   * @param x m/s
   * @param y m/s
   * @param omega rad/s
   */
  private void driveSafely(double x, double y, double omega) {
    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    omega = omegaLimiter.calculate(omega);

    if (this.config.enabled()) {
      var spds = ChassisSpeeds.discretize(x, y, omega, Robot.kDefaultPeriod);
      this.drive.drive(spds.vxMetersPerSecond, spds.vyMetersPerSecond, spds.omegaRadiansPerSecond);
    }
  }

  /**
   * The primary method for controlling the drivebase. The provided parameters specify the
   * robot-relative chassis speeds of the robot.
   *
   * <p>This method is responsible for applying safety code to prevent the robot from attempting to
   * exceed its physical limits both in terms of speed and acceleration.
   *
   * <p>
   *
   * @param x m/s
   * @param y m/s
   * @param omega rad/s
   */
  public final void driveRobotOriented(double x, double y, double omega) {
    Telemetry.drive.fieldOrientedVelocityX = 0;
    Telemetry.drive.fieldOrientedVelocityY = 0;
    Telemetry.drive.fieldOrientedVelocityOmega = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseX = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseY = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseHeading = 0;

    driveSafely(x, y, omega);
  }

  /**
   * Translate ChassisSpeed into raw vX, vY, vR for PathPlanner
   *
   * @param speeds ChassisSpeed object
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    driveRobotOriented(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * Translate double[] speeds into ChassisSpeed for PathPlanner
   *
   * @return ChassisSpeeds containing the robot's relative speeds
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    double[] speeds = drive.getMeasuredRobotVelocity();

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    chassisSpeeds.vxMetersPerSecond = speeds[0];
    chassisSpeeds.vyMetersPerSecond = speeds[1];
    chassisSpeeds.omegaRadiansPerSecond = speeds[2];

    return chassisSpeeds;
  }

  /** Stop all motors as fast as possible */
  public void stop() {
    driveRobotOriented(0, 0, 0);
  }

  /**
   * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
   * field-oriented inputs into the required robot-oriented inputs that can be used by the robot.
   *
   * @param x the linear velocity of the robot in metres per second. Positive x is away from the
   *     blue alliance wall
   * @param y the linear velocity of the robot in metres per second. Positive y is to the left of
   *     the robot
   * @param omega the rotation rate of the heading of the robot in radians per second. CCW positive.
   */
  public final void driveFieldOriented(double x, double y, double omega) {
    Telemetry.drive.fieldOrientedDeltaToPoseX = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseY = 0;
    Telemetry.drive.fieldOrientedDeltaToPoseHeading = 0;

    driveFieldOrientedInternal(x, y, omega);
  }

  /*
   * INTERNAL method for driving field-oriented. This should be called by another method that
   * updates
   * telemetry values fieldOrientedDeltaToPoseX, fieldOrientedDeltaToPoseY,
   * fieldOrientedDeltaToPoseHeading.
   *
   * @param x
   *
   * @param y
   *
   * @param omega
   */
  private void driveFieldOrientedInternal(double x, double y, double omega) {
    Telemetry.drive.fieldOrientedVelocityX = x;
    Telemetry.drive.fieldOrientedVelocityY = y;
    Telemetry.drive.fieldOrientedVelocityOmega = omega;

    var robotOriented = SwerveMath.toRobotOriented(x, y, Math.toRadians(drive.getYaw()));
    driveSafely(robotOriented[0], robotOriented[1], omega);
  }

  /**
   * Lock the swerve drive to prevent it from moving. This can only be called when the robot is
   * nearly stationary.
   *
   * @return true if successfully locked, false otherwise
   */
  public boolean lock() {
    return drive.lock();
  }

  public double getUltrasonicDistanceM() {
    return ultrasonicDistanceM;
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return drive.getPose(); // todo: fixme:
  }

  public double getYaw() {
    return drive.getYaw();
  }

  public double getYawRate() {
    return drive.getYawRate();
  }

  /**
   * Set the gyro yaw offset of the robot, in degrees.
   *
   * @param yaw the yaw offset of the robot, in degrees
   */
  public void setYaw(double yaw) {
    drive.setYaw(yaw);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    drive.zeroGyro();
  }

  /**
   * Change the robot's internal understanding of its position and rotation. This is not an
   * incremental change or suggestion, it discontinuously re-sets the pose to the specified pose.
   *
   * @param pose the new location and heading of the robot.
   */
  public void resetOdometry(Pose2d pose) {
    drive.resetOdometry(pose);
  }

  /**
   * Set the desired module state for the named module. This should ONLY be used when testing the
   * serve drivebase in a controlled environment.
   *
   * <p>This SHOULD NOT be called during normal operation - it is designed for TEST MODE ONLY!
   *
   * @param moduleName the module to activate
   * @param speed in m/s
   * @param angle in degrees
   */
  public void setModuleState(String moduleName, double speed, double angle) {
    drive.setModuleState(moduleName, speed, angle);
  }

  @Override
  public String toString() {
    Pose2d pose = getPose();
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getDegrees();

    StringBuilder sb = new StringBuilder();
    sb.append(this.getClass().getSimpleName())
        .append(": Pose: ")
        .append(Math.round(x * 100d) / 100d)
        .append(",")
        .append(Math.round(y * 100d) / 100d)
        .append(",")
        .append(Math.round(theta * 10d) / 10d);

    return sb.toString();
  }

  /*
   * *********************************************************************************************
   * Convenience methods for subsystem users
   */

  /**
   * Compute the required rotation speed of the robot given the desired heading. Note the desired
   * heading is specified in degrees, adn the returned value is in radians per second.
   *
   * @param desiredHeadingDegrees the desired heading of the robot
   * @return the required rotation speed of the robot (omega) in rad/s
   */
  public double computeOmega(double desiredHeadingDegrees) {
    return computeOmega(desiredHeadingDegrees, config.rotationConfig().defaultRotVelocityRadPS());
  }

  /**
   * Compute the required rotation speed of the robot given the desired heading. Note the desired
   * heading is specified in degrees, adn the returned value is in radians per second.
   *
   * @param desiredHeadingDegrees the desired heading of the robot
   * @param maxOmegaRadPerSec the maximum allowable rotation speed of the robot
   * @return the required rotation speed of the robot (omega) in rad/s
   */
  public double computeOmega(double desiredHeadingDegrees, double maxOmegaRadPerSec) {
    double omega = headingPIDController.calculate(drive.getYaw(), desiredHeadingDegrees);
    return Math.min(omega, maxOmegaRadPerSec);
  }

  public double computeTranslateVelocity(double distance, double maxSpeedMPS, double tolerance) {
    final double decelZoneMetres = 1.2;
    final double verySlowZone = 0.2;
    final double verySlowSpeed = 0.15;
    //    double maxSpeedMPS = Constants.Swerve.TRANSLATION_CONFIG.maxSpeedMPS();
    //    maxSpeedMPS = 3.5;
    double speed;

    final double absDist = Math.abs(distance);

    if (absDist < tolerance) {
      return 0;
    }
    // Very slow zone
    if (absDist < verySlowZone) {
      speed = verySlowSpeed;
    }
    // Full speed ahead!
    else if (absDist >= decelZoneMetres) {
      speed = maxSpeedMPS;
    }
    // Decel Zone
    else {
      speed = (absDist / decelZoneMetres) * maxSpeedMPS;
    }

    speed *= Math.signum(distance);

    return speed;
  }

  public Translation2d computeTranslateVelocity2024(
      Translation2d translationToTravel,
      double maxSpeed,
      double tolerance,
      boolean driveThrough,
      double decelDistance) {

    double distanceMetres = translationToTravel.getNorm();
    double verySlowSpeed = 0.15;

    // don't worry about tiny translations
    if (distanceMetres < tolerance) {
      return new Translation2d();
    }

    // safety code
    if (maxSpeed > TRANSLATION_CONFIG.maxSpeedMPS()) {
      maxSpeed = TRANSLATION_CONFIG.maxSpeedMPS();
    }

    // ensure that we have enough room to decelerate
    double decelDistRatio = distanceMetres / decelDistance;

    if (!driveThrough) {
      if (decelDistRatio < 1) {
        maxSpeed *= decelDistRatio;
      }
    }

    double speed;
    if (distanceMetres >= decelDistance || driveThrough) {
      // cruising
      speed = maxSpeed;
    } else {
      // decelerating
      double pctToGo = distanceMetres / decelDistance;
      speed = maxSpeed * pctToGo * 1.2;
    }

    // Confirm speed is not too slow to move
    if (speed < verySlowSpeed) {
      speed = verySlowSpeed;
    }

    Rotation2d angle = translationToTravel.getAngle();

    double xSign = Math.signum(translationToTravel.getX());
    double ySign = Math.signum(translationToTravel.getY());
    return new Translation2d(
        xSign * speed * Math.abs(angle.getCos()), ySign * speed * Math.abs(angle.getSin()));
  }

  public double getClosestReefAngle(double currentX, double currentY) {
    double blueReefX = 4.49;
    double redReefX = 13.06;
    double reefY = 4.03;

    double rrX;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      rrX = redReefX - currentX;
    } else {
      rrX = blueReefX - currentX;
    }
    double rrY = reefY - currentY;

    double angleAroundReefDeg =
        SwerveUtils.normalizeDegrees(Math.toDegrees(Math.atan2(rrX, rrY)) + 90);

    if (angleAroundReefDeg > -150 && angleAroundReefDeg < -90) {
      return -60;
    } else if (angleAroundReefDeg < -30 && angleAroundReefDeg > -90) {
      return -120;
    } else if (angleAroundReefDeg < 30 && angleAroundReefDeg > -30) {
      return 180;
    } else if (angleAroundReefDeg < 90 && angleAroundReefDeg > 30) {
      return 120;
    } else if (angleAroundReefDeg < 150 && angleAroundReefDeg > 90) {
      return 60;
    } else {
      return 0;
    }
  }
}
