package frc.robot.commands.coral;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class SetupScoreCommandBotPose extends LoggingCommand {

  /*
  Get distance with ultrasonic sensor
  Move toward desired distance
  While moving move the elevator and arm to position
  */
  private final CoralSubsystem coralSubsystem;
  private final SwerveSubsystem swerve;
  private final Constants.CoralConstants.CoralPose coralPose;
  private final LimelightVisionSubsystem limelightVisionSubsystem;


  private boolean atElevatorHeight = false;
  private boolean atArmAngle = false;
  private final Constants.AutoConstants.FieldLocation location;
  private final double targetHeadingDeg;
  private boolean doneDriving = false;




  private double vX = 0;
  private double vY = 0;

  public SetupScoreCommandBotPose(
      CoralSubsystem coralSubsystem,
      SwerveSubsystem swerve,
      LimelightVisionSubsystem limelightVisionSubsystem,
      Constants.AutoConstants.FieldLocation location,
      Constants.CoralConstants.CoralPose coralPose
      ) {

    this.coralSubsystem = coralSubsystem;
    this.swerve = swerve;
    this.limelightVisionSubsystem = limelightVisionSubsystem;
    this.location = location;
    this.targetHeadingDeg = SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees());
    this.coralPose = coralPose;

    addRequirements(coralSubsystem);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {

    logCommandStart();

    atElevatorHeight = false;
    atArmAngle = false;
    log("Pose: " + swerve.getPose());
  }

  @Override
  public void execute() {
    atElevatorHeight = coralSubsystem.moveElevatorToHeight(coralPose.elevatorHeight);
    atArmAngle = coralSubsystem.moveArmToAngle(coralPose.armAngle);

    Pose2d currentPose = swerve.getPose();

    double xDif = location.pose.getX() - currentPose.getX();
    double yDif = location.pose.getY() - currentPose.getY();

    double angleDif = SwerveUtils.normalizeDegrees(targetHeadingDeg - currentPose.getRotation().getDegrees());
//        log("Xdif: " + xDif + " Ydif: " + yDif + " ºdif: " + angleDif);

//    swerve.driveFieldOriented(
//            swerve.computeTranslateVelocity(xDif, 0.02, 0.5),
//            swerve.computeTranslateVelocity(yDif, 0.02, 0.5),
//            swerve.computeOmega(targetHeadingDeg));
    swerve.driveFieldOriented(
            calcSwerveSpeedX(xDif, 0.02, 0.5),
            calcSwerveSpeedY(yDif, 0.02, 0.5),
            swerve.computeOmega(targetHeadingDeg));
//    System.out.println("xSpeed: " + swerve.computeTranslateVelocity(xDif, 0.02, 0.5));
//    System.out.println("ySpeed: " + swerve.computeTranslateVelocity(yDif, 0.02, 0.5));



  }

  private double calcSwerveSpeedX(double xDif, double tolerance, double maxSpeed) {
    double ySpeed = Math.max(0.2, Math.min(maxSpeed, xDif));
    if (xDif < tolerance) {
      ySpeed = 0;
    }
    return ySpeed;
  }

  private double calcSwerveSpeedY(double yDif, double tolerance, double maxSpeed) {
    double xSpeed = Math.max(0.15, Math.min(maxSpeed, yDif));
    if (yDif < tolerance) {
      xSpeed = 0;
    } return xSpeed;
  }

  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
    swerve.stop();
    coralSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    if (!coralSubsystem.isCoralDetected()) {
      return true;
    }
    if (atElevatorHeight && atArmAngle && doneDriving) {
      return true;
    }
    return false;
  }
}
