package frc.robot.commands.coral;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightBotPose;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class SetupScoreCommand extends LoggingCommand {

  /*
  Get distance with ultrasonic sensor
  Move toward desired distance
  While moving move the elevator and arm to position
  */
  private final CoralSubsystem coralSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final LimelightVisionSubsystem limelightVisionSubsystem;
  private final Constants.CoralConstants.CoralPose coralPose;
  private final Constants.CoralConstants.DesiredDistanceToTargetCM distanceToTarget;

  private boolean atElevatorHeight = false;
  private boolean atArmAngle = false;
  private boolean atCorrectDistanceForward = false;
  private boolean atCorrectDistanceLeft = false;
  private boolean isLeftBranch;

  private double deltaDistanceForward;
  private double deltaDistanceLeft;
  private double currentDistance;

  private double tX;
  private double offsetFromTag;

  private double vX = 0;
  private double vY = 0;

  public SetupScoreCommand(
      Constants.CoralConstants.CoralPose coralPose,
      Constants.CoralConstants.DesiredDistanceToTargetCM desiredDistanceToTarget,
      boolean isLeftBranch,
      CoralSubsystem coralSubsystem,
      SwerveSubsystem swerveSubsystem,
      LimelightVisionSubsystem limelightVisionSubsystem) {

    this.coralSubsystem = coralSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.limelightVisionSubsystem = limelightVisionSubsystem;
    this.coralPose = coralPose;
    this.distanceToTarget = desiredDistanceToTarget;
    this.isLeftBranch = isLeftBranch;

    addRequirements(coralSubsystem);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {

    logCommandStart();

    atElevatorHeight = false;
    atArmAngle = false;
    atCorrectDistanceForward = false;
  }

  @Override
  public void execute() {
    atElevatorHeight = coralSubsystem.moveElevatorToHeight(coralPose.elevatorHeight);
    atArmAngle = coralSubsystem.moveArmToAngle(coralPose.armAngle);
//
//    currentDistance = coralSubsystem.getUltrasonicDistanceCm();
//    deltaDistanceForward = currentDistance - distanceToTarget.getDistance();
//
//    tX = limelightVisionSubsystem.angleToTarget();
//    offsetFromTag = currentDistance * Math.tan(Math.toRadians(tX));
//
//    if (limelightVisionSubsystem.getTagAmount() > 0.5) {
//      if (isLeftBranch) {
//        deltaDistanceLeft = Constants.CoralConstants.OFFSET_FROM_TAG_FOR_SCORING - offsetFromTag;
//      } else {
//        deltaDistanceLeft = -Constants.CoralConstants.OFFSET_FROM_TAG_FOR_SCORING - offsetFromTag;
//      }
//    } else {
//      deltaDistanceLeft = 0;
//    }
//
//    if (Math.abs(deltaDistanceForward) < Constants.CoralConstants.SCORING_DISTANCE_TOLERANCE) {
//      atCorrectDistanceForward = true;
//    } else {
//      atCorrectDistanceForward = false;
//    }
//
//    if (Math.abs(deltaDistanceLeft) < Constants.CoralConstants.SCORING_DISTANCE_TOLERANCE) {
//      atCorrectDistanceLeft = true;
//    } else {
//      atCorrectDistanceLeft = false;
//    }
//
//    if (!atCorrectDistanceLeft) {
//      vY = 0.25 * Math.signum(deltaDistanceLeft);
//    } else {
//      vY = 0;
//    }
//
//    if (!atCorrectDistanceForward && atCorrectDistanceLeft) {
//      vX = 0.25 * Math.signum(deltaDistanceForward);
//    } else {
//      vX = 0;
//    }
//
//      swerveSubsystem.driveRobotOriented(vX, vY, 0);
//
//    SmartDashboard.putNumber("1310/SetupScoreCommand/offSetFromTag ", offsetFromTag);
//    SmartDashboard.putNumber("1310/SetupScoreCommand/deltaDistanceLeft", deltaDistanceLeft);
//    SmartDashboard.putBoolean("1310/SetupScoreCommand/atCorrectDistanceForward", atCorrectDistanceForward);
//    SmartDashboard.putBoolean("1310/SetupScoreCommand/atCorrectDistanceLeft", atCorrectDistanceLeft);





  }

  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerveSubsystem.stop();
    coralSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    if (!coralSubsystem.isCoralDetected()) {
      return true;
    }
    if (atElevatorHeight && atArmAngle && atCorrectDistanceForward) {
      return true;
    }
    return false;
  }
}
