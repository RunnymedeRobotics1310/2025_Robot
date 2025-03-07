package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.DriveInlineWithTagCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.SetGyroCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Emergency1CoralAutoCommand extends SequentialCommandGroup {

  public Emergency1CoralAutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision) {

    double headingOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      headingOffset = 180;
    }

    addCommands(new SetGyroCommand(swerve, 180 + headingOffset));

    addCommands(
        new WaitCommand(1)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, 1, 0, 180 + headingOffset)));

    addCommands(
        new DriveInlineWithTagCommand(swerve, vision)
            .alongWith(
                new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L4, coral)));
    addCommands(
        new WaitCommand(0.5)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, 0, 0.25, 180 + headingOffset)));

    addCommands(
        new WaitCommand(1)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, 0.75, 0, 180 + headingOffset)));

    addCommands(new PlantCoralCommand(coral));

    addCommands(
        new WaitCommand(0.25)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, -1, 0, 180 + headingOffset)));

    addCommands(new WaitCommand(1));

    addCommands(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.COMPACT, coral));
  }
}
