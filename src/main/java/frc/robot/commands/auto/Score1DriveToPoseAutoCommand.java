package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToScorePositionCommand;
import frc.robot.commands.swervedrive.NullDriveCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score1DriveToPoseAutoCommand extends SequentialCommandGroup {

  public Score1DriveToPoseAutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision, double delay) {

    addCommands(new WaitCommand(delay));

    addCommands(
        new DriveToFieldLocationCommand(
            swerve, Constants.AutoConstants.FieldLocation.preScoreBlueLeft1));

    addCommands(
        new DriveToScorePositionCommand(swerve, vision, 18, true)
            .alongWith(
                new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L4, coral)));

    addCommands(new WaitCommand(1));

    addCommands(new PlantCoralCommand(coral).deadlineFor(new NullDriveCommand(swerve)));
  }
}
