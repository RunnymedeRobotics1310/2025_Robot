package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedOmegaCommand;
import frc.robot.commands.swervedrive.DriveToVisibleTagCommand;
import frc.robot.commands.swervedrive.NullDriveCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class AlignShootLeaveCommand extends SequentialCommandGroup {

    public AlignShootLeaveCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision, CoralSubsystem coral, Constants.CoralConstants.CoralPose coralPose, boolean isLeftBranch) {

        Pose2d startPose = swerve.getPose();

        addCommands(
            new DriveToVisibleTagCommand(swerve, vision, isLeftBranch)
                .alongWith(new MoveToCoralPoseCommand(coralPose, coral)));
        addCommands(new PlantCoralCommand(coral));

    addCommands(
        new DriveRobotOrientedOmegaCommand(swerve, -0.5, 0, 0)
            .withTimeout(0.2)
            .andThen(
                new WaitCommand(0.2)
                    //                        .andThen(new
                    // MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.CLOSE_INTAKE,
                    // coral))
                    .andThen(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.FAR_INTAKE, coral))));
  }
}
