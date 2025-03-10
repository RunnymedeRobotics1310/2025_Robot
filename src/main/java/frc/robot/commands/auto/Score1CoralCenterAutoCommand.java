package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score1CoralCenterAutoCommand extends SequentialCommandGroup {

  public Score1CoralCenterAutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision, double delay) {

    addCommands(new WaitCommand(delay));

    addCommands(new SetAutoGyroCommand(swerve, 180));

    addCommands(
        new DriveToFieldLocationCommand(
            swerve, Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_6));
    //    addCommands(
    //        new SetupScoreCommand(
    //            Constants.CoralConstants.CoralPose.SCORE_L4,
    //            Constants.CoralConstants.DesiredDistanceToTargetCM.LEVEL_4,
    //                false,
    //            coral,
    //            swerve,
    //                vision));
    addCommands(
        new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L4, coral)
            .deadlineFor(new NullDriveCommand(swerve)));

    addCommands(
        new WaitCommand(1)
            .deadlineFor(
                new DriveRobotOrientedCommand(
                    swerve,
                    0.35,
                    0,
                    Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_6
                        .pose
                        .getRotation()
                        .getDegrees())));
    addCommands(new PlantCoralCommand(coral).deadlineFor(new NullDriveCommand(swerve)));
  }
}
