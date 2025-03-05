package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_X;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_Y;

import ca.team1310.swerve.core.config.*;
import ca.team1310.swerve.utils.Coordinates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.swerve.SwerveDriveSubsystemConfig;
import frc.robot.subsystems.swerve.SwerveRotationConfig;
import frc.robot.subsystems.swerve.SwerveTranslationConfig;
import frc.robot.subsystems.vision.VisionConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose.
 *
 * <p>All constants should be declared globally (i.e. public static). <br>
 * Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;
  public static final VisionConfig VISION_CONFIG =
      new VisionConfig(
          0,
          0,
          FieldConstants.FIELD_EXTENT_METRES_X,
          FieldConstants.FIELD_EXTENT_METRES_Y,
          0.7,
          0.3,
          .5,
          true);

  public enum BotTarget {
    // Blue Field Targets
    BLUE_AMP(new Translation3d(1.8415, 8.2042, 0.873252)),
    BLUE_SOURCE(new Translation3d(15.632176, 0.564896, 0)),
    BLUE_SPEAKER(new Translation3d(0.0381, 5.547868, 2.124202)),
    BLUE_STAGE(new Translation3d(4.86791, 4.105656, 1.6764)),

    // Red Field Targets
    RED_AMP(new Translation3d(14.700758, 8.2042, 0.873252)),
    RED_SOURCE(new Translation3d(0.908812, 0.564769, 0)),
    RED_SPEAKER(new Translation3d(16.579342, 5.547868, 2.124202)),
    RED_STAGE(new Translation3d(11.676634, 4.105656, 1.6764)),

    // Blue Side Notes
    BLUE_NOTE_WOLVERINE(new Translation3d(2.9, 4.11, 0)),
    BLUE_NOTE_BARNUM(new Translation3d(2.9, 5.5, 0)),
    BLUE_NOTE_VALJEAN(new Translation3d(2.9, 7, 0)),

    // Red Side Notes
    RED_NOTE_WOLVERINE(new Translation3d(13.53, 4.11, 0)),
    RED_NOTE_BARNUM(new Translation3d(13.53, 5.5, 0)),
    RED_NOTE_VALJEAN(new Translation3d(13.53, 7, 0)),

    // Centre Field Notes
    CENTRE_NOTE_1(new Translation3d(8.16, 0.75, 0)),
    CENTRE_NOTE_2(new Translation3d(8.16, 2.43, 0)),
    CENTRE_NOTE_3(new Translation3d(8.16, 4.11, 0)),
    CENTRE_NOTE_4(new Translation3d(8.16, 5.79, 0)),
    CENTRE_NOTE_5(new Translation3d(8.16, 7.47, 0)),

    // When No Target is Set
    NONE(new Translation3d(0, 0, 0)),

    // No focus, but go to any tag visible
    ALL(new Translation3d(0, 0, 0));

    private final Translation3d location;

    BotTarget(Translation3d location) {
      this.location = location;
    }

    public Translation3d getLocation() {
      return location;
    }

    @Override
    public String toString() {
      return "BotTarget: " + name() + " at " + location;
    }
  }

  public static final class OiConstants {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = .2;

    /**
     * Standard drive speed factor. Regular teleop drive will use this factor of the max
     * translational speed.
     */
    public static final double GENERAL_SPEED_FACTOR = .65;

    /**
     * Maximum drive speed factor. When boosting, this factor will be multiplied against the max
     * translational speed.
     */
    public static final double MAX_SPEED_FACTOR = 1;

    /**
     * Slow mode drive speed factor. When running in slow mode, this factor will be multiplied
     * against the max translational speed.
     */
    public static final double SLOW_SPEED_FACTOR = .05;
  }

  public static final class FieldConstants {

    public static final double FIELD_EXTENT_METRES_Y = 8.211;
    public static final double FIELD_EXTENT_METRES_X = 16.541;

    public static final double[] TARGET_HEADINGS =
        new double[] {
          126, // 1
          -126, // 2
          90, // 3
          0, // 4
          0, // 5
          -60, // 6
          0, // 7
          60, // 8
          120, // 9
          180, // 10
          -120, // 11
          -126, // 12
          126, // 13
          0, // 14
          0, // 15
          -90, // 16
          60, // 17
          0, // 18
          -60, // 19
          -120, // 20
          180, // 21
          120 // 22
        };
  }

  public static final class Swerve {

    /** Front to back from the middle of the wheels */
    public static final double WHEEL_BASE_METRES = inchesToMeters(23);

    /** Side to side from the middle of the wheels */
    public static final double TRACK_WIDTH_METRES = inchesToMeters(26.5);

    public static final double SDS_MK4I_WHEEL_RADIUS_M = 0.051;

    public static final SwerveTranslationConfig TRANSLATION_CONFIG =
        new SwerveTranslationConfig(
            /* tolerance (m) */ 0.02,
            /* min speed (m/s) */ 1.0,
            /* max speed (m/s) */ 4.8,
            /* max module speed (m/s) */ 5.36,
            /* max acceleration (m/s/s) */ 42.0,
            /* velocity PID p */ 1.2,
            /* velocity PID i */ 0,
            /* velocity PID d */ 0);

    public static final SwerveRotationConfig ROTATION_CONFIG =
        new SwerveRotationConfig(
            /* max rot vel (rad/s) */ Rotation2d.fromRotations(1).getRadians(),
            /* max rotation accel (rad/s/s) */ Rotation2d.fromRotations(4).getRadians(),
            /* heading PID p */ 0.025, // Rads/Deg
            /* heading PID i */ 0,
            /* heading PID d */ 0);

    private static final MotorConfig ANGLE_MOTOR_CONFIG =
        new MotorConfig(
            /* motor hardware type */ MotorType.NEO_SPARK_MAX,
            /* inverted? */ true,
            /* current limit (A) */ 20,
            /* nominal voltage (V) */ 12,
            /* ramp rate 0 to full power (s) */ 0.25,
            /* angle motor gear ratio */ 150.0 / 7 /* SDS MK4i 150/7:1 */,
            /* angle motor PID p */ 0.0125,
            /* angle motor PID i */ 0,
            /* angle motor PID d */ 0,
            /* angle motor PID ff */ 0,
            /* angle motor PID izone */ 0);

    private static final MotorConfig DRIVE_MOTOR_CONFIG =
        new MotorConfig(
            /* motor hardware type */ MotorType.NEO_SPARK_FLEX,
            /* inverted? */ false,
            /* current limit (A) */ 40,
            /* current limit (A) */ 12,
            /* ramp rate 0 to full power (s) */ 0.25,
            /* drive motor gear ratio */ 6.75 /* SDS MK4i L2 --> 6.75:1 */,
            /* drive motor PID p */ 0.11,
            /* drive motor PID i */ 0,
            /* drive motor PID d */ 0,
            /* drive motor PID ff */ 0,
            /* drive motor PID izone */ 0);

    private static final EncoderConfig ANGLE_ENCODER_CONFIG = new EncoderConfig(false, 0.005, 5);

    public static final ModuleConfig FRONT_LEFT =
        new ModuleConfig(
            "frontleft",
            new Coordinates(TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2),
            SDS_MK4I_WHEEL_RADIUS_M,
            10,
            DRIVE_MOTOR_CONFIG,
            11,
            ANGLE_MOTOR_CONFIG,
            12,
            Rotation2d.fromRotations(0.257324).getDegrees(),
            ANGLE_ENCODER_CONFIG);

    public static final ModuleConfig FRONT_RIGHT =
        new ModuleConfig(
            "frontright",
            new Coordinates(TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2),
            SDS_MK4I_WHEEL_RADIUS_M,
            20,
            DRIVE_MOTOR_CONFIG,
            21,
            ANGLE_MOTOR_CONFIG,
            22,
            Rotation2d.fromRotations(0.431152).getDegrees(),
            ANGLE_ENCODER_CONFIG);

    public static final ModuleConfig BACK_LEFT =
        new ModuleConfig(
            "backleft",
            new Coordinates(-TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2),
            SDS_MK4I_WHEEL_RADIUS_M,
            35,
            DRIVE_MOTOR_CONFIG,
            36,
            ANGLE_MOTOR_CONFIG,
            37,
            Rotation2d.fromRotations(-0.360596).getDegrees(),
            ANGLE_ENCODER_CONFIG);

    public static final ModuleConfig BACK_RIGHT =
        new ModuleConfig(
            "backright",
            new Coordinates(-TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2),
            SDS_MK4I_WHEEL_RADIUS_M,
            30,
            DRIVE_MOTOR_CONFIG,
            31,
            ANGLE_MOTOR_CONFIG,
            32,
            Rotation2d.fromRotations(0.274658).getDegrees(),
            ANGLE_ENCODER_CONFIG);

    public static final CoreSwerveConfig CORE_SWERVE_CONFIG =
        new CoreSwerveConfig(
            WHEEL_BASE_METRES,
            TRACK_WIDTH_METRES,
            SDS_MK4I_WHEEL_RADIUS_M,
            Robot.kDefaultPeriod,
            TRANSLATION_CONFIG.maxModuleSpeedMPS(),
            TRANSLATION_CONFIG.maxSpeedMPS(),
            ROTATION_CONFIG.maxRotVelocityRadPS(),
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT,
            TelemetryLevel.VERBOSE);

    public static final SwerveDriveSubsystemConfig SUBSYSTEM_CONFIG =
        new SwerveDriveSubsystemConfig(
            true, CORE_SWERVE_CONFIG, TRANSLATION_CONFIG, ROTATION_CONFIG);
  }

  public static final class UsefulPoses {

    public static final Pose2d SCORE_BLUE_AMP =
        (new Pose2d(BotTarget.BLUE_AMP.getLocation().getX(), 7.6, Rotation2d.fromDegrees(90)));
    public static final Pose2d SCORE_RED_AMP =
        (new Pose2d(BotTarget.RED_AMP.getLocation().getX(), 7.6, Rotation2d.fromDegrees(90)));

    public static final Pose2d BLUE_2_2_20 = new Pose2d(2, 2, Rotation2d.fromDegrees(20));
    public static final Pose2d RED_2_2_20 = new Pose2d(14.54, 2, Rotation2d.fromDegrees(-20));
  }

  public static final class AutoConstants {

    public enum AutoPattern {
      DO_NOTHING,
      EXIT_ZONE,
      OPTIMISTIC_AUTO
    }

    public enum Delay {
      NO_DELAY,
      WAIT_0_5_SECOND,
      WAIT_1_SECOND,
      WAIT_1_5_SECONDS,
      WAIT_2_SECONDS,
      WAIT_2_5_SECONDS,
      WAIT_3_SECONDS,
      WAIT_5_SECONDS
    }

    public enum FieldLocation {
      preScoreBlueLeft1(new Pose2d(2.8126, 4.1909, Rotation2d.fromDegrees(0))),
      preScoreBlueLeft2(new Pose2d(3.5085, 5.3948, Rotation2d.fromDegrees(300))),
      preScoreBlueLeft3(new Pose2d(3.7943, 5.5598, Rotation2d.fromDegrees(300))),
      preScoreBlueLeft4(new Pose2d(5.1843, 5.5598, Rotation2d.fromDegrees(240))),
      preScoreBlueLeft5(new Pose2d(5.4701, 5.3948, Rotation2d.fromDegrees(240))),
      preScoreBlueLeft6(new Pose2d(6.1660, 4.1909, Rotation2d.fromDegrees(180))),
      preScoreBlueRight1(new Pose2d(2.8126, 3.8609, Rotation2d.fromDegrees(0))),
      preScoreBlueRight2(new Pose2d(3.5085, 2.6570, Rotation2d.fromDegrees(60))),
      preScoreBlueRight3(new Pose2d(3.92, 2.58, Rotation2d.fromDegrees(60))),
      preScoreBlueRight4(new Pose2d(5.1843, 2.4920, Rotation2d.fromDegrees(120))),
      preScoreBlueRight5(new Pose2d(5.4701, 2.6570, Rotation2d.fromDegrees(120))),
      preScoreBlueRight6(new Pose2d(6.1660, 3.8609, Rotation2d.fromDegrees(180))),
      preIntakeCentreLeftBlueStation(new Pose2d(1.139, 7.000, Rotation2d.fromDegrees(126))),
      preIntakeCentreRightBlueStation(new Pose2d(1.139, 1.052, Rotation2d.fromDegrees(234))),
      // Pickup Locations
      redRightOuterStation(
          new Pose2d(
              FIELD_EXTENT_METRES_X - 100,
              FIELD_EXTENT_METRES_Y - 70,
              Rotation2d.fromDegrees(234))),
      blueRightOuterStation(new Pose2d(100, 70, Rotation2d.fromDegrees(234))),

      blueLeftOuterStation(new Pose2d(1.15, 7.03, Rotation2d.fromDegrees(-234))),

      // Reef Score Locations (Lettered as seen in manual - counter-clockwise starting from
      // close-left)
      redA(
          new Pose2d(
              FieldConstants.FIELD_EXTENT_METRES_X - 3.20,
              FieldConstants.FIELD_EXTENT_METRES_Y - 4.10,
              Rotation2d.fromDegrees(0))),
      blueA(new Pose2d(3.20, 4.10, Rotation2d.fromDegrees(0))),
      redB(
          new Pose2d(
              FieldConstants.FIELD_EXTENT_METRES_X - 3.20,
              FieldConstants.FIELD_EXTENT_METRES_Y - 3.80,
              Rotation2d.fromDegrees(0))),
      blueB(new Pose2d(3.20, 3.80, Rotation2d.fromDegrees(0))),
      redC(
          new Pose2d(
              FieldConstants.FIELD_EXTENT_METRES_X - 4.00,
              FieldConstants.FIELD_EXTENT_METRES_Y - 3.10,
              Rotation2d.fromDegrees(60))),
      blueC(new Pose2d(4.00, 3.10, Rotation2d.fromDegrees(60))),
      redD(
          new Pose2d(
              FieldConstants.FIELD_EXTENT_METRES_X - 4.20,
              FieldConstants.FIELD_EXTENT_METRES_Y - 2.80,
              Rotation2d.fromDegrees(60))),
      blueD(new Pose2d(4.20, 2.80, Rotation2d.fromDegrees(60))),
      redE(
          new Pose2d(
              FieldConstants.FIELD_EXTENT_METRES_X - 5.10,
              FieldConstants.FIELD_EXTENT_METRES_Y - 2.70,
              Rotation2d.fromDegrees(120))),
      blueE(new Pose2d(5.10, 2.70, Rotation2d.fromDegrees(120))),
      blueJ(new Pose2d(5.10, FIELD_EXTENT_METRES_Y - 2.70, Rotation2d.fromDegrees(-120))),
      blueK(new Pose2d(4.20, FIELD_EXTENT_METRES_Y - 2.80, Rotation2d.fromDegrees(-60))),
      blueL(new Pose2d(4.00, FIELD_EXTENT_METRES_Y - 3.10, Rotation2d.fromDegrees(-60))),

      // Auto transit poses
      redRightExitTransit(
          new Pose2d(
              FIELD_EXTENT_METRES_X - 4.40,
              FIELD_EXTENT_METRES_Y - 0.90,
              Rotation2d.fromDegrees(180))),
      blueRightExitTransit(new Pose2d(4.40, 0.90, Rotation2d.fromDegrees(180))),
      redLeftExitTransit(
          new Pose2d(
              FIELD_EXTENT_METRES_X - 5.00,
              FIELD_EXTENT_METRES_Y - 6.00,
              Rotation2d.fromDegrees(0))),
      blueLeftExitTransit(new Pose2d(4.40, 6.50, Rotation2d.fromDegrees(0))),

      redRightPickupTransit(
          new Pose2d(
              FIELD_EXTENT_METRES_X - 2.80,
              FIELD_EXTENT_METRES_Y - 1.70,
              Rotation2d.fromDegrees(-135))),
      blueRightPickupTransit(new Pose2d(2.80, 1.70, Rotation2d.fromDegrees(-135))),
      blueLeftPickupTransit(
          new Pose2d(2.80, FIELD_EXTENT_METRES_Y - 2.22, Rotation2d.fromDegrees(135)));

      public final Pose2d pose;

      FieldLocation(Pose2d pose) {
        this.pose = pose;
      }
    }
  }

  public static final class DriveConstants {

    // NOTE: Follower motors are at CAN_ID+1
    public static final int LEFT_MOTOR_CAN_ID = 10;
    public static final int RIGHT_MOTOR_CAN_ID = 20;
    public static final boolean LEFT_MOTOR_INVERTED = false;
    public static final boolean RIGHT_MOTOR_INVERTED = true;
    public static final double CM_PER_ENCODER_COUNT = 3.503;
    public static final boolean GYRO_INVERTED = false;

    /** Proportional gain for gyro pid tracking */
    public static final double GYRO_PID_KP = 0.01;

    public static final double DRIVE_SCALING_BOOST = 1;
    public static final double DRIVE_SCALING_NORMAL = .6;
    public static final double DRIVE_SCALING_SLOW = .3;

    public static enum DriveMode {
      TANK,
      ARCADE,
      SINGLE_STICK_LEFT,
      SINGLE_STICK_RIGHT;
    }
  }

  public static final class CoralConstants {

    /*
     * Motor CAN IDs and inversions
     */
    public static final int ELEVATOR_MOTOR_CAN_ID = 40;
    public static final int ARM_MOTOR_CAN_ID = 41;
    public static final int INTAKE_MOTOR_CAN_ID = 42;
    public static final boolean ELEVATOR_MOTOR_INVERTED = true;
    public static final boolean ARM_MOTOR_INVERTED = false;
    public static final boolean INTAKE_MOTOR_INVERTED = false;
    // Ultrasonic port
    public static final int ULTRASONIC_SENSOR_PORT = 0;
    public static final double ULTRASONIC_M = (609.6 - 30.5) / (2.245 - 0.12);
    public static final double ULTRASONIC_B = 609.6 - ULTRASONIC_M * 2.245;
    public static final double SCORING_DISTANCE_TOLERANCE = 5;
    /*
     * Elevator Constants
     */
    public static final double ELEVATOR_MAX_SPEED = 1;
    public static final double ELEVATOR_MAX_HEIGHT = 188;
    public static final double ELEVATOR_TOLERANCE = 2.5;
    public static final double ELEVATOR_P = 0.05;
    // Maximum manual tuning speed
    public static final double ELEVATOR_TUNE_MAX_SPEED = 0.2;
    // Safety constants near the limits
    public static final double ELEVATOR_SLOW_ZONE_SPEED = 0.2;
    public static final double ELEVATOR_SLOW_ZONE = 10; // encoder counts
    public static final double ELEVATOR_METERS_PER_ENCODER_COUNT = 0.745 / 122.13;
    public static final double THOMAS_STARTING_HEIGHT = 0.88; // m
    /*
     * Arm Constants
     */
    public static final double ARM_MAX_SPEED = 0.6;
    public static final double ARM_LOWER_LIMIT_POSITION = 0;
    public static final double ARM_UPPER_LIMIT_POSITION = 126;
    public static final double ARM_CAMERA_THRESHOLD_POSITION = 100;
    public static final boolean ARM_ANGLE_ENCODER_INVERTED = false;
    // Set the encoder offset so that the encoder reads 0.1 rotations against the hard stop
    // This is so that the angle can go negative instead of back to 360 deg when slightly
    // less than zero. This constant was read off the REV Hardware Client Absolute Encoder page
    public static final double ARM_ANGLE_ENCODER_OFFSET = 0.0296887;
    // Maximum manual tuning speed
    public static final double ARM_TUNE_MAX_SPEED = 0.2;
    // Pseudo PID and safe zone constants
    public static final double ARM_ANGLE_TOLERANCE = 1.5;
    public static final double ARM_FAST_SPEED = 0.5;
    public static final double ARM_SLOW_ZONE_SPEED = 0.15;
    public static final double ARM_SLOW_ZONE_ANGLE = 10;
    /*
     * Intake Constants
     */
    public static final double CORAL_INTAKE_SPEED = 0.5;
    public static final double CORAL_OUTAKE_SPEED = 0.8;
    public static final double CORAL_EJECT_SPEED = 0.7;
    public static final double PLANT_ROTATIONS = 15;

    // Elevator Heights in encoder counts
    public enum ElevatorHeight {
      COMPACT(0),
      CLOSE_INTAKE(47.5),
      FAR_INTAKE(21),
      LEVEL_1(0),
      LEVEL_2(0),
      LEVEL_3(72),
      LEVEL_4(ELEVATOR_MAX_HEIGHT),
      REMOVE_LOW_ALGAE(60),
      REMOVE_HIGH_ALGAE(130);

      public final double encoderCount;

      ElevatorHeight(double encoderCount) {
        this.encoderCount = encoderCount;
      }
    }

    // Arm Angles in degrees
    public enum ArmAngle {
      COMPACT(-1),
      INTAKE(20),
      LEVEL_1(0),
      LEVEL_2(120),
      LEVEL_3(120),
      LEVEL_4(112),
      REMOVE_ALGAE(40);

      public final double angle;

      ArmAngle(double angle) {
        this.angle = angle;
      }
    }

    public enum DesiredDistanceToTargetCM {
      INTAKE(20),
      LEVEL_1(20),
      LEVEL_2(20),
      LEVEL_3(20),
      LEVEL_4(26),
      REMOVE_ALGAE(20);

      public final double distance;

      DesiredDistanceToTargetCM(double distance) {
        this.distance = distance;
      }

      public double getDistance() {
        return distance;
      }
    }

    //How far to the left or right you need to be from a reef tag to score
    public static final double OFFSET_FROM_TAG_FOR_SCORING = 14;

    public enum CoralPose {
      COMPACT(ElevatorHeight.COMPACT, ArmAngle.COMPACT),
      CLOSE_INTAKE(ElevatorHeight.CLOSE_INTAKE, ArmAngle.INTAKE),
      FAR_INTAKE(ElevatorHeight.FAR_INTAKE, ArmAngle.INTAKE),
      SCORE_L1(ElevatorHeight.LEVEL_1, ArmAngle.LEVEL_1),
      SCORE_L2(ElevatorHeight.LEVEL_2, ArmAngle.LEVEL_2),
      SCORE_L3(ElevatorHeight.LEVEL_3, ArmAngle.LEVEL_3),
      SCORE_L4(ElevatorHeight.LEVEL_4, ArmAngle.LEVEL_4),
      REMOVE_LOW_ALGAE(ElevatorHeight.REMOVE_LOW_ALGAE, ArmAngle.REMOVE_ALGAE),
      REMOVE_HIGH_ALGAE(ElevatorHeight.REMOVE_HIGH_ALGAE, ArmAngle.REMOVE_ALGAE);

      public final ElevatorHeight elevatorHeight;
      public final ArmAngle armAngle;

      CoralPose(ElevatorHeight elevatorHeight, ArmAngle armAngle) {
        this.elevatorHeight = elevatorHeight;
        this.armAngle = armAngle;
      }
    }

  }

  public static final class ClimbConstants {
    public static final int CLIMB_FORWARDS_PNEUMATIC_PORT = 7;
    public static final int CLIMB_REVERSE_PNEUMATIC_PORT = 0;
  }

  public static final class AlgaeConstants {
    public static final int ALGAE_FORWARDS_PNEUMATIC_PORT = 6;
    public static final int ALGAE_REVERSE_PNEUMATIC_PORT = 1;
    public static final int INTAKE_MOTOR_CAN_ID = 50;

    public static final int ALGAE_INTAKE_SPEED = 1;
    public static final int ALGAE_OUTTAKE_SPEED = -1;
  }

  public static final class LightsConstants {

    public static final int LED_STRING_PWM_PORT = 9;
    public static final int LED_STRING_LENGTH = 60;
  }
}
