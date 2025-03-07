package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToScorePositionCommand;
import frc.robot.commands.swervedrive.NullDriveCommand;
import frc.robot.commands.swervedrive.SetGyroCommand;
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
      double delay) {

    double headingOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      headingOffset = 180;
    }

    addCommands(new SetGyroCommand(swerve, 180 + headingOffset));

    addCommands(new WaitCommand(delay));

    addCommands(new DriveToFieldLocationCommand(swerve, fieldLocation));

    addCommands(
        new DriveToScorePositionCommand(swerve, vision, fieldLocation, true)
            .alongWith(new MoveToCoralPoseCommand(coralPose, coral)));

    addCommands(new WaitCommand(0.5));

    addCommands(new PlantCoralCommand(coral).deadlineFor(new NullDriveCommand(swerve)));
  }
}
