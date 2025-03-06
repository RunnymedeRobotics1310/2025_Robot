package frc.robot.commands.swervedrive;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class DriveToScorePositionCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem visionSubsystem;
  private final Constants.AutoConstants.FieldLocation location;
  private final double targetHeadingDeg;
  private final boolean isLeftBranch;

  private Pose2d initialRobotPose;
  private Pose2d targetPose;
  private int tagId = 0;

  public static final double OFFSET_FROM_TAG_FOR_SCORING = 0.14;
  public static final double OFFSET_FROM_TAG_ROBOT_HALF_LENGTH = 0.50;

  public DriveToScorePositionCommand(
      SwerveSubsystem swerve,
      LimelightVisionSubsystem visionSubsystem,
      Constants.AutoConstants.FieldLocation location,
      boolean isLeftBranch) {
    this.swerve = swerve;
    this.visionSubsystem = visionSubsystem;
    this.location = location;
    this.targetHeadingDeg = SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees());
    this.isLeftBranch = isLeftBranch;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    logCommandStart();

    // Setup initial pose and target based on target tag
    initialRobotPose = swerve.getPose();
    computeTarget(initialRobotPose, true);
  }

  @Override
  public void execute() {

    // Get current pose, and recalculate our current real world pose based on target tag
    Pose2d currentPose = swerve.getPose();
    computeTarget(currentPose, false);

    double xDif = targetPose.getX() - currentPose.getX();
    double yDif = targetPose.getY() - currentPose.getY();

    double xS = calcSwerveSpeed(xDif, 0.02, 0.2, 1);
    double yS = calcSwerveSpeed(yDif, 0.02, 0.2, 1);

    swerve.driveFieldOriented(xS, yS, swerve.computeOmega(targetHeadingDeg));
  }

  private void computeTarget(Pose2d currentPose, boolean initalize) {

    if (initalize) {
      // Whatever tag we're looking at, it's the one we want.
      tagId = (int) visionSubsystem.getVisibleTargetTagId();
      visionSubsystem.setTargetTagId(tagId);
    } else {
      // Need to make sure target tag is in view to do this
      if (!visionSubsystem.isTagInView(tagId)) {
        return;
      }
    }

    double robotHeading = currentPose.getRotation().getDegrees();
    double targetAngleRelative = -visionSubsystem.angleToTarget();
    double distanceToTarget = visionSubsystem.distanceToTarget();

    // Compute target position relative to robot
    double targetGlobalAngle = robotHeading + targetAngleRelative;
    double targetX = distanceToTarget * Math.cos(Math.toRadians(targetGlobalAngle));
    double targetY = distanceToTarget * Math.sin(Math.toRadians(targetGlobalAngle));

    Pose2d newPose =
        new Pose2d(
            location.pose.getX() - targetX,
            location.pose.getY() - targetY,
            currentPose.getRotation());

    swerve.resetOdometry(newPose);

    if (initalize) {
      double sideOffset = isLeftBranch ? OFFSET_FROM_TAG_FOR_SCORING : -OFFSET_FROM_TAG_FOR_SCORING;

      // Compute side offset along target's orientation (Perpendicular)
      double sideOffsetX = sideOffset * Math.cos(Math.toRadians(targetHeadingDeg + 90));
      double sideOffsetY = sideOffset * Math.sin(Math.toRadians(targetHeadingDeg + 90));

      // Compute backward offset away from the target (Opposite direction)
      double backwardOffsetX =
          OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.cos(Math.toRadians(targetHeadingDeg + 180));
      double backwardOffsetY =
          OFFSET_FROM_TAG_ROBOT_HALF_LENGTH * Math.sin(Math.toRadians(targetHeadingDeg + 180));

      // Compute final target position with offset
      double finalX = targetX + sideOffsetX + backwardOffsetX;
      double finalY = targetY + sideOffsetY + backwardOffsetY;

      targetPose =
          new Pose2d(
              newPose.getX() + finalX,
              newPose.getY() + finalY,
              Rotation2d.fromDegrees(targetHeadingDeg));

      SmartDashboard.putNumber("1310/SetupScoreCommand/finalX", finalX);
      SmartDashboard.putNumber("1310/SetupScoreCommand/finalY", finalY);
    }

    SmartDashboard.putNumber("1310/SetupScoreCommand/targetX", targetX);
    SmartDashboard.putNumber("1310/SetupScoreCommand/targetY", targetY);
    SmartDashboard.putNumber("1310/SetupScoreCommand/targetAngle", targetGlobalAngle);
    SmartDashboard.putNumber("1310/SetupScoreCommand/targetTagId", tagId);
    SmartDashboard.putString("1310/SetupScoreCommand/initalPose", initialRobotPose.toString());
    SmartDashboard.putString("1310/SetupScoreCommand/myPose", newPose.toString());
    SmartDashboard.putString("1310/SetupScoreCommand/targetPose", targetPose.toString());
  }

  private double calcSwerveSpeed(
      double distance, double tolerance, double minSpeed, double maxSpeed) {
    double xSpeed = Math.max(minSpeed, Math.min(maxSpeed, Math.abs(distance)));
    if (Math.abs(distance) < tolerance) {
      xSpeed = 0;
    }
    return xSpeed * Math.signum(distance);
  }

  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
  }

  @Override
  public boolean isFinished() {
    boolean done =
        (SwerveUtils.isCloseEnough(
                swerve.getPose().getTranslation(), targetPose.getTranslation(), 0.02)
            && SwerveUtils.isCloseEnough(swerve.getYaw(), targetHeadingDeg, 2));
    if (done) {
      System.out.println(
          "REACHED DESTINATION: x["
              + swerve.getPose().getX()
              + "] y["
              + swerve.getPose().getY()
              + "], deg["
              + swerve.getPose().getRotation().getDegrees()
              + "]");
      visionSubsystem.setTargetTagId(0);
    }
    return done;
  }
}
