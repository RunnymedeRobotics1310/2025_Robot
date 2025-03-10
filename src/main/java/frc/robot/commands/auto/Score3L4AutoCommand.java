package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.FieldLocation.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Score3L4AutoCommand extends SequentialCommandGroup {

  double speed = 3;

  public Score3L4AutoCommand(SwerveSubsystem swerve, double delay) {
    super();

    addCommands(new SetAutoGyroCommand(swerve, 180));

    addCommands(new SetPoseCommand(swerve, new Pose2d(7.2, 5.63, Rotation2d.fromDegrees(0))));
    //        addCommands(new ZeroGyroCommand(swerve));

    addCommands(new WaitCommand(delay));

    addCommands(new DriveToFieldLocationCommand(swerve, PRE_SCORE_LEFT_4));
    // Score l4 at left4
    addCommands(new WaitCommand(2).deadlineFor(new NullDriveCommand(swerve)));
    //        addCommands(new DriveThroughFieldLocationCommand(swerve, blueLeftExitTransit, speed));
    addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
    // Intake coral
    //        addCommands(new DriveThroughFieldLocationCommand(swerve, blueLeftPickupTransit,
    // speed));
    addCommands(new DriveToFieldLocationCommand(swerve, PRE_SCORE_LEFT_2));
    // Score left1
    addCommands(new WaitCommand(2).deadlineFor(new NullDriveCommand(swerve)));
    // addCommands(new DriveThroughFieldLocationCommand(swerve, blueLeftPickupTransit, speed));
    addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
    // Intake coral
    addCommands(new DriveToFieldLocationCommand(swerve, PRE_SCORE_LEFT_3));
    // Score coral
    addCommands(new WaitCommand(2).deadlineFor(new NullDriveCommand(swerve)));
    addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation));
    // Intake coral

  }
}
