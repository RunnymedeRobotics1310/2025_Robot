package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.coral.SetupScoreCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Score1CoralCenterAutoCommand extends SequentialCommandGroup {

  public Score1CoralCenterAutoCommand(SwerveSubsystem swerve, CoralSubsystem coral, double delay) {

    addCommands(new WaitCommand(delay));

    addCommands(
        new DriveToFieldLocationCommand(
            swerve, Constants.AutoConstants.FieldLocation.preScoreBlueLeft6));
    addCommands(
        new SetupScoreCommand(
            Constants.CoralConstants.CoralPose.SCORE_L4,
            Constants.CoralConstants.DesiredDistanceToTargetCM.LEVEL_4,
            coral,
            swerve));
    //        addCommands(new PlantCoralCommand(coral));
    //        addCommands(new SetCoralPoseCommand(coral, compact));

  }
}
