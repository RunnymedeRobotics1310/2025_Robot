package frc.robot.commands.auto;

import edu.wpi.first.math.proto.Plant;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.DriveInlineWithTagCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Emergency1CoralAutoCommand extends SequentialCommandGroup {

  public Emergency1CoralAutoCommand(SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision) {

      addCommands(new WaitCommand(1)
              .deadlineFor(new DriveRobotOrientedCommand(swerve, 1, 0, 180)));

    addCommands(new DriveInlineWithTagCommand(swerve, vision)
            .alongWith(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L4, coral)));
    addCommands(new WaitCommand(0.5)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, 0, 0.25, 180)));

    addCommands(new WaitCommand(1).deadlineFor(new DriveRobotOrientedCommand(swerve, 0.75, 0, 180)));

    addCommands(new PlantCoralCommand(coral));

    addCommands(new WaitCommand(0.25)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, -1, 0, 180)));
    addCommands(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.COMPACT, coral));
  }
}
