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

public class Score1DriveToPoseAutoCommand extends SequentialCommandGroup {

  public Score1DriveToPoseAutoCommand(
      SwerveSubsystem swerve,
      CoralSubsystem coral,
      LimelightVisionSubsystem vision,
      Constants.AutoConstants.FieldLocation fieldLocation,
      Constants.CoralConstants.CoralPose coralPose,
      double delay,
      boolean driveToPrePose,
      double startHeading) {

    addCommands(new SetAutoGyroCommand(swerve, startHeading));

    addCommands(new WaitCommand(delay));

    if (driveToPrePose) {
      // addCommands(new DriveRobotOrientedOmegaCommand(swerve, 1.00, 0.00, 0).withTimeout(0.4));
      addCommands(new DriveToFieldLocationCommand(swerve, fieldLocation));
    }

    addCommands(
        new DriveToTagCommand(swerve, vision, fieldLocation, true)
            .alongWith(new MoveToCoralPoseCommand(coralPose, coral).withTimeout(2)));

    addCommands(new WaitCommand(0.5));

    addCommands(new PlantCoralCommand(coral).deadlineFor(new NullDriveCommand(swerve)));

    addCommands(new DriveRobotOrientedOmegaCommand(swerve, 0.00, 0.20, 0).withTimeout(1));

    //    addCommands(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.COMPACT,
    // coral));
  }
}
