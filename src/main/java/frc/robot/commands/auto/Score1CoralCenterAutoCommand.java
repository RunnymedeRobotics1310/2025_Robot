package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.NullDriveCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score1CoralCenterAutoCommand extends SequentialCommandGroup {

  public Score1CoralCenterAutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision, double delay) {

    addCommands(new WaitCommand(delay));

    addCommands(
        new DriveToFieldLocationCommand(
            swerve, Constants.AutoConstants.FieldLocation.preScoreBlueRight4));
    //    addCommands(
    //        new SetupScoreCommand(
    //            Constants.CoralConstants.CoralPose.SCORE_L4,
    //            Constants.CoralConstants.DesiredDistanceToTargetCM.LEVEL_4,
    //                false,
    //            coral,
    //            swerve,
    //                vision));
    addCommands(
        new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L2, coral)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, 0.35, 0, 0)));
    addCommands(new PlantCoralCommand(coral).deadlineFor(new NullDriveCommand(swerve)));

    //    //        addCommands(new PlantCoralCommand(coral));
    //    //        addCommands(new SetCoralPoseCommand(coral, compact));

  }
}
