package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_X;
import static frc.robot.Constants.FieldConstants.FIELD_EXTENT_METRES_Y;

import ca.team1310.swerve.core.config.*;
import ca.team1310.swerve.utils.Coordinates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveDriveSubsystemConfig;
import frc.robot.subsystems.swerve.SwerveRotationConfig;
import frc.robot.subsystems.swerve.SwerveTranslationConfig;
import frc.robot.subsystems.vision.VisionConfig;
import frc.robot.subsystems.vision.VisionTelemetryLevel;
import java.util.HashMap;
import java.util.Map;

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

  public static final class TelemetryConfig {
    public static boolean drive = true;
    public static VisionTelemetryLevel vision = VisionTelemetryLevel.NONE;
    public static TelemetryLevel swerve = TelemetryLevel.VERBOSE;
    public static boolean test = false;
    public static boolean oi = false;
    public static boolean coral = false;
    public static boolean climb = false;
    public static boolean pneumatics = false;
  }

  public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;

  public static final class RobotConfig {
    public static final double LENGTH_METRES = 0.81;
    public static final double WIDTH_METRES = 0.71;
    public static final double HEIGHT_METRES = 1.016;
    public static final double BUMPER_WIDTH = 0.089;
  }

  public static final class VisionConstants {
    public static final VisionConfig VISION_CONFIG =
        new VisionConfig(0, 0, 0.7, 0.1, .5, true, Constants.TelemetryConfig.vision);

    public static final String VISION_PRIMARY_LIMELIGHT_NAME = "nikola";
    public static final String VISION_SECONDARY_LIMELIGHT_NAME = "thomas";

    public static final double LIMELIGHT_OFFSET_FROM_FRONT_BUMPER = 0.23;
  }

  public static final class OiConstants {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = .2;

    /**
     * Standard drive speed factor. Regular teleop drive will use this factor of the max
     * translational speed.
     */
    public static final double GENERAL_SPEED_FACTOR = .5;

    /**
     * Maximum drive speed factor. When boosting, this factor will be multiplied against the max
     * translational speed.
     */
    public static final double MAX_SPEED_FACTOR = 1;

    /**
     * Slow mode drive speed factor. When running in slow mode, this factor will be multiplied
     * against the max translational speed.
     */
    public static final double SLOW_SPEED_FACTOR = .1;

    /**
     * Operator tune factor for tuning robot oriented at reef. This factor will be multiplied
     * against the max translational speed.
     */
    public static final double OPERATOR_SPEED_FACTOR = 0.1;
  }

  public static final class FieldConstants {

    public static final double FIELD_EXTENT_METRES_Y = 8.052;
    public static final double FIELD_EXTENT_METRES_X = 17.55;

    // This is physical tag locations on field, from 2025FieldDrawings.pdf but the heading is
    // swapped 180 degrees to indicate heading to face the tag, vs the orientation the tag is facing
    public enum TAGS {
      RED_LEFT_SOURCE(1, new Pose2d(16.697198, 0.655320, Rotation2d.fromDegrees(-54))),
      RED_RIGHT_SOURCE(2, new Pose2d(16.697198, 7.396480, Rotation2d.fromDegrees(54))),
      RED_PROCESSOR(3, new Pose2d(11.560810, 8.055610, Rotation2d.fromDegrees(90))),
      RED_RIGHT_BARGE(4, new Pose2d(9.276080, 6.137656, Rotation2d.fromDegrees(180))),
      RED_LEFT_BARGE(5, new Pose2d(9.276080, 1.914906, Rotation2d.fromDegrees(180))),
      RED_LEFT_REEF_2_3(6, new Pose2d(13.474446, 3.306318, Rotation2d.fromDegrees(120))),
      RED_LEFT_RIGHT_REEF_1(7, new Pose2d(13.890498, 4.025900, Rotation2d.fromDegrees(180))),
      RED_RIGHT_REEF_2_3(8, new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(-120))),
      RED_RIGHT_REEF_4_5(9, new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(-60))),
      RED_RIGHT_LEFT_REEF_6(10, new Pose2d(12.227306, 4.025900, Rotation2d.fromDegrees(0))),
      RED_LEFT_REEF_4_5(11, new Pose2d(12.643358, 3.306318, Rotation2d.fromDegrees(60))),
      BLUE_RIGHT_SOURCE(12, new Pose2d(0.851154, 0.655320, Rotation2d.fromDegrees(-126))),
      BLUE_LEFT_SOURCE(13, new Pose2d(0.851154, 7.396480, Rotation2d.fromDegrees(126))),
      BLUE_LEFT_BARGE(14, new Pose2d(8.272272, 6.137656, Rotation2d.fromDegrees(0))),
      BLUE_RIGHT_BARGE(15, new Pose2d(8.272272, 1.914906, Rotation2d.fromDegrees(0))),
      BLUE_PROCESSOR(16, new Pose2d(5.987542, -0.003810, Rotation2d.fromDegrees(-90))),
      BLUE_RIGHT_REEF_2_3(17, new Pose2d(4.073906, 3.306318, Rotation2d.fromDegrees(60))),
      BLUE_RIGHT_LEFT_REEF_1(18, new Pose2d(3.657600, 4.025900, Rotation2d.fromDegrees(0))),
      BLUE_LEFT_REEF_2_3(19, new Pose2d(4.073906, 4.745482, Rotation2d.fromDegrees(-60))),
      BLUE_LEFT_REEF_4_5(20, new Pose2d(4.904740, 4.745482, Rotation2d.fromDegrees(-120))),
      BLUE_LEFT_RIGHT_REEF_6(21, new Pose2d(5.321046, 4.025900, Rotation2d.fromDegrees(180))),
      BLUE_RIGHT_REEF_4_5(22, new Pose2d(4.904740, 3.306318, Rotation2d.fromDegrees(120)));

      private static final Map<Integer, TAGS> lookup = new HashMap<>();

      static {
        for (TAGS tag : TAGS.values()) {
          lookup.put(tag.tagId, tag);
        }
      }

      public final int tagId;
      public final Pose2d pose;

      TAGS(int tagId, Pose2d pose) {
        this.tagId = tagId;
        this.pose = pose;
      }

      public static TAGS getTagById(int tagId) {
        return lookup.get(tagId);
      }

      public static boolean isValidTagId(int tagId) {
        return lookup.containsKey(tagId);
      }
    }
  }

  public static final class Swerve {

    /** Front to back from the middle of the wheels */
    public static final double WHEEL_BASE_METRES = inchesToMeters(26.5);

    /** Side to side from the middle of the wheels */
    public static final double TRACK_WIDTH_METRES = inchesToMeters(23);

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
            /* max rot vel (rad/s) */ Rotation2d.fromRotations(1.5).getRadians(),
            /* defaultRotVelocityRadPS (rad/s) */ Rotation2d.fromRotations(1).getRadians(),
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
            Rotation2d.fromRotations(0.255859).getDegrees(),
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
            Rotation2d.fromRotations(0.969727).getDegrees(),
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
            Rotation2d.fromRotations(0.635498).getDegrees(),
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
            Rotation2d.fromRotations(0.279297).getDegrees(),
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
            Constants.TelemetryConfig.swerve);

    public static final SwerveDriveSubsystemConfig SUBSYSTEM_CONFIG =
        new SwerveDriveSubsystemConfig(
            true, CORE_SWERVE_CONFIG, TRANSLATION_CONFIG, ROTATION_CONFIG, TelemetryConfig.drive);

    // Ultrasonic port
    public static final int ULTRASONIC_SENSOR_PORT = 0;
    public static final double SCORING_DISTANCE_TOLERANCE = 0.05;
  }

  public static final class AutoConstants {

    public enum AutoPattern {
      DO_NOTHING,
      EXIT_ZONE,
      SCORE_3_LEFT,
      SCORE_3_RIGHT,
      SCORE_1_CENTER,
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

    public enum ReefLocation1 {
      LEFT_1,
      LEFT_2,
      LEFT_3,
      LEFT_4,
      LEFT_5,
      LEFT_6,
      RIGHT_1,
      RIGHT_2,
      RIGHT_3,
      RIGHT_4,
      RIGHT_5,
      RIGHT_6,
    }

    public enum ReefLocation2 {
      LEFT_1,
      LEFT_2,
      LEFT_3,
      LEFT_4,
      LEFT_5,
      LEFT_6,
      RIGHT_1,
      RIGHT_2,
      RIGHT_3,
      RIGHT_4,
      RIGHT_5,
      RIGHT_6,
    }

    public enum ReefLocation3 {
      LEFT_1,
      LEFT_2,
      LEFT_3,
      LEFT_4,
      LEFT_5,
      LEFT_6,
      RIGHT_1,
      RIGHT_2,
      RIGHT_3,
      RIGHT_4,
      RIGHT_5,
      RIGHT_6,
    }

    public enum ReefPosition1 {
      CORAL_L4,
      CORAL_L3,
      CORAL_L2,
    }

    public enum ReefPosition2 {
      CORAL_L4,
      CORAL_L3,
      CORAL_L2,
    }

    public enum ReefPosition3 {
      CORAL_L4,
      CORAL_L3,
      CORAL_L2,
    }

    public enum FieldLocation {
      // Generalized Multi Alliance Locations
      PRE_SCORE_LEFT_1(new Pose2d(2.8126, 4.1909, Rotation2d.fromDegrees(0)), 18, 7, true),
      PRE_SCORE_LEFT_2(new Pose2d(3.33, 5.7, Rotation2d.fromDegrees(300)), 19, 6, false),
      PRE_SCORE_LEFT_3(new Pose2d(3.62, 5.85, Rotation2d.fromDegrees(300)), 19, 6, true),
      PRE_SCORE_LEFT_4(new Pose2d(5.3843, 5.5598, Rotation2d.fromDegrees(240)), 20, 11, false),
      PRE_SCORE_LEFT_5(new Pose2d(5.4701, 5.3948, Rotation2d.fromDegrees(240)), 20, 11, true),
      PRE_SCORE_LEFT_6(new Pose2d(6.1660, 4.16, Rotation2d.fromDegrees(180)), 21, 10, false),
      PRE_SCORE_RIGHT_1(new Pose2d(2.8126, 3.8609, Rotation2d.fromDegrees(0)), 18, 7, false),
      PRE_SCORE_RIGHT_2(new Pose2d(3.25, 2.23, Rotation2d.fromDegrees(60)), 17, 8, true),
      PRE_SCORE_RIGHT_3(new Pose2d(3.7943, 2.64, Rotation2d.fromDegrees(60)), 17, 8, false),
      PRE_SCORE_RIGHT_4(new Pose2d(5.1843, 2.5, Rotation2d.fromDegrees(120)), 22, 9, true),
      PRE_SCORE_RIGHT_5(new Pose2d(5.4701, 2.6570, Rotation2d.fromDegrees(120)), 22, 9, false),
      PRE_SCORE_RIGHT_6(new Pose2d(6.1660, 3.8609, Rotation2d.fromDegrees(180)), 21, 10, true),
      PRE_INTAKE_CENTRE_LEFT_STATION(new Pose2d(1.139, 7.000, Rotation2d.fromDegrees(126)), 13, 1),
      PRE_INTAKE_CENTRE_RIGHT_STATION(new Pose2d(1.139, 1.052, Rotation2d.fromDegrees(234)), 12, 2),

      AUTO_START_LEFT(new Pose2d(7, 5.81, Rotation2d.fromDegrees(180))),
      AUTO_START_RIGHT(new Pose2d(7, 2.242, Rotation2d.fromDegrees(180))),

      // Legacy Alliance Specific Locations
      blueRightOuterStation(new Pose2d(1.15, 1.02, Rotation2d.fromDegrees(234)), 12, 2),

      blueLeftOuterStationFromFar(new Pose2d(1.45, 6.73, Rotation2d.fromDegrees(-234)), 13, 1),
      blueLeftOuterStation(new Pose2d(1.02, 6.93, Rotation2d.fromDegrees(-234)), 13, 1),

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
      public final int blueTagId;
      public final int redTagId;
      public final boolean isLeftSide;

      FieldLocation(Pose2d pose) {
        this(pose, 0, 0, false);
      }

      FieldLocation(Pose2d pose, int blueTagId, int redTagId) {
        this(pose, blueTagId, redTagId, false);
      }

      FieldLocation(Pose2d pose, int blueTagId, int redTagId, boolean isLeftSide) {
        this.pose = pose;
        this.blueTagId = blueTagId;
        this.redTagId = redTagId;
        this.isLeftSide = isLeftSide;
      }
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
    /*
     * Elevator Constants
     */
    public static final double ELEVATOR_MAX_SPEED = 1;
    public static final double ELEVATOR_MAX_HEIGHT = 162;
    public static final double ELEVATOR_TOLERANCE = 1;
    public static final double ELEVATOR_MAX_SLEW = 0.2;
    public static final double ELEVATOR_P = 0.05;
    public static final double ELEVATOR_HOLD_STEADY_SPEED = 0.02;
    // Maximum manual tuning speed
    public static final double ELEVATOR_TUNE_MAX_SPEED = 0.2;
    // Safety constants near the limits
    public static final double ELEVATOR_SLOW_ZONE_SPEED = 0.25;
    public static final double ELEVATOR_SLOW_ZONE = 15; // encoder counts
    public static final double ELEVATOR_METERS_PER_ENCODER_COUNT =
        0.745 / 122.13; // TODO: fixme: WRONG
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
    public static final double ARM_ANGLE_ENCODER_OFFSET = 0.3506509;
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
    public static final double CORAL_INTAKE_SPEED = 0.8;
    public static final double CORAL_OUTAKE_SPEED = 0.8;
    public static final double CORAL_OUTAKE_SLOW_SPEED = -0.25;
    public static final double CORAL_EJECT_SPEED = 0.7;
    public static final double PLANT_ROTATIONS = 30;
    public static final double POST_SHOT_WAIT_TIME = 0.1;

    // Elevator Heights in encoder counts
    public enum ElevatorHeight {
      COMPACT(0),
      CLOSE_INTAKE(30),
      FAR_INTAKE(19.4),
      LEVEL_1(0),
      LEVEL_2(0),
      LEVEL_3(53),
      LEVEL_4(ELEVATOR_MAX_HEIGHT),
      REMOVE_LOW_ALGAE(16.5),
      REMOVE_HIGH_ALGAE(78);

      public final double encoderCount;

      ElevatorHeight(double encoderCount) {
        this.encoderCount = encoderCount;
      }
    }

    // Arm Angles in degrees
    public enum ArmAngle {
      COMPACT(1.5),
      CLOSE_INTAKE(40.0),
      FAR_INTAKE(35),
      LEVEL_1(33),
      LEVEL_2(120),
      LEVEL_3(126),
      LEVEL_4(112),
      REMOVE_ALGAE(74);

      public final double angle;

      ArmAngle(double angle) {
        this.angle = angle;
      }
    }

    public enum CoralPose {
      COMPACT(ElevatorHeight.COMPACT, ArmAngle.COMPACT),
      CLOSE_INTAKE(ElevatorHeight.CLOSE_INTAKE, ArmAngle.CLOSE_INTAKE),
      FAR_INTAKE(ElevatorHeight.FAR_INTAKE, ArmAngle.FAR_INTAKE),
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
    public static final int CLIMB_FORWARDS_PNEUMATIC_PORT = 6;
    public static final int CLIMB_REVERSE_PNEUMATIC_PORT = 0;
    public static final int CLIMB_SENSOR_DIO_PORT = 9;
  }

  public static final class LightsConstants {

    public static final int LED_STRING_PWM_PORT = 7;
    public static final int LED_STRING_LENGTH = 60;
  }
}
