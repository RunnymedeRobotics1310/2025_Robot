package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.SetPoseCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToLeftCenterPointAutoCommand extends SequentialCommandGroup {

  public DriveToLeftCenterPointAutoCommand(SwerveSubsystem swerve) {
    addCommands(new SetPoseCommand(swerve, new Pose2d(7.6, 6.00, Rotation2d.fromDegrees(0))));
    addCommands(new ZeroGyroCommand(swerve));

    addCommands(
        new DriveToFieldLocationCommand(
            swerve, Constants.AutoConstants.FieldLocation.blueLeftExitTransit));
  }
}
