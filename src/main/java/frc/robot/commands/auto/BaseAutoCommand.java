package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.Constants.CoralConstants.CoralPose.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class BaseAutoCommand extends SequentialCommandGroup {

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;
  private final CoralSubsystem coral;
  private double allianceOffset = 0;
  private final double speed = 3.5;

  public BaseAutoCommand(
      SwerveSubsystem swerve, LimelightVisionSubsystem vision, CoralSubsystem coral) {
    this.swerve = swerve;
    this.vision = vision;
    this.coral = coral;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      allianceOffset = 180;
    }
  }

  protected Command driveToLocation(FieldLocation location) {
    return new DriveToFieldLocationCommand(swerve, location);
  }

  protected Command driveThroughToLocation(
      FieldLocation location, double speed, double toleranceM, double decelDistance) {
    return new DriveThroughFieldLocationCommand(
        swerve, location, speed, false, toleranceM, decelDistance);
  }

  protected Command driveThroughToLocation(FieldLocation location, double speed) {
    return new DriveThroughFieldLocationCommand(swerve, location, speed, false);
  }

  protected Command driveThroughLocation(FieldLocation location, double speed) {
    return new DriveThroughFieldLocationCommand(swerve, location, speed, true);
  }

  protected Command setCoralPose(CoralPose pose) {
    return new MoveToCoralPoseCommand(pose, coral);
  }

  protected Command plant() {
    return new PlantCoralCommand(coral, false);
  }

  protected Command approachReef(FieldLocation location) {
    return new DriveToReefTagCommand(swerve, vision, location).alongWith(setCoralPose(SCORE_L4));
  }

  public Command goScoreL4Coral(FieldLocation location) {
    double locationHeading = location.pose.getRotation().getDegrees() + allianceOffset;

    return driveThroughLocation(location, speed)
        .andThen(approachReef(location))
        .andThen(plant())
        .andThen(new DriveRobotOrientedCommand(swerve, -0.5, 0, locationHeading).withTimeout(0.1))
        .andThen(setCoralPose(COMPACT));
  }

  public Command scoreL4CoralStop2(FieldLocation location, double speed) {
    return (driveThroughLocation(location, speed).deadlineFor(setCoralPose(SCORE_L3)))
        .andThen(approachReef(location))
        .andThen(plant());
  }

  public Command scoreL4CoralStop(FieldLocation location, double speed) {
    return (driveThroughToLocation(location, speed).deadlineFor(setCoralPose(SCORE_L4)))
        .andThen(approachReef(location))
        .andThen(plant());
  }

  public Command autoIntake(FieldLocation location) {
    double locationHeading = location.pose.getRotation().getDegrees() + allianceOffset;

    return driveThroughLocation(location, speed)
        .andThen(new DriveIntoWallCommand(swerve, 0.25, 0, locationHeading))
        .alongWith(new IntakeCoralCommand(coral, false));
  }

  public Command scoreL4CoralAndIntake2(
      FieldLocation reefLocation, FieldLocation intakeLocation, double speed) {
    double reefHeading = reefLocation.pose.getRotation().getDegrees() + allianceOffset;
    double intakeHeading = intakeLocation.pose.getRotation().getDegrees() + allianceOffset;

    return scoreL4CoralStop2(reefLocation, speed)
        .andThen(new DriveRobotOrientedCommand(swerve, -0.5, 0, reefHeading).withTimeout(0.2))
        .andThen(
            (new WaitCommand(0.2).andThen(new IntakeCoralCommand(coral, false)))
                .deadlineFor(
                    driveThroughToLocation(intakeLocation, speed, 0.2, 0.6)
                        .andThen(new DriveIntoWallCommand(swerve, 0.25, 0, intakeHeading))));
  }

  public Command scoreL4CoralAndIntake(
      FieldLocation reefLocation, FieldLocation intakeLocation, double speed) {
    double reefHeading = reefLocation.pose.getRotation().getDegrees() + allianceOffset;
    double intakeHeading = intakeLocation.pose.getRotation().getDegrees() + allianceOffset;

    return scoreL4CoralStop(reefLocation, speed)
        .andThen(new DriveRobotOrientedCommand(swerve, -0.5, 0, reefHeading).withTimeout(0.2))
        .andThen(
            (new WaitCommand(0.4).andThen(new IntakeCoralCommand(coral, false)))
                .deadlineFor(
                    driveThroughToLocation(intakeLocation, speed)
                        .andThen(new DriveIntoWallCommand(swerve, 0.25, 0, intakeHeading))));
  }
}
