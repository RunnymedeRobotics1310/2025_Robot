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

    addCommands(
            new DriveToVisibleTagCommand(swerve, vision, true)
                    .alongWith(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L4, coral).withTimeout(2)));

    addCommands(new WaitCommand(0.5));
    addCommands(new PlantCoralCommand(coral).deadlineFor(new NullDriveCommand(swerve)));

    addCommands(new DriveRobotOrientedOmegaCommand(swerve, 0.00, 0.20, 0).withTimeout(1));
    addCommands(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.COMPACT, coral));
  }
}
