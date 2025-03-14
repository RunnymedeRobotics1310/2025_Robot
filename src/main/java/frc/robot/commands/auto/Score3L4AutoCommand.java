package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.FieldLocation.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score3L4AutoCommand extends SequentialCommandGroup {

  double speed = 3;
  double allianceOffset = 0;

  public Score3L4AutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision, double delay) {
    super();

    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      allianceOffset = 180;
    }

    addCommands(new SetAutoGyroCommand(swerve, 180));
    addCommands(new WaitCommand(delay));

    addCommands(new DriveToFieldLocationCommand(swerve, PRE_SCORE_LEFT_4));

    addCommands(new DriveToVisibleTagCommand(swerve, vision, false)
            .alongWith(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L4, coral)));

    addCommands(new WaitCommand(0.5));
    addCommands(new PlantCoralCommand(coral).deadlineFor(new NullDriveCommand(swerve)));

    addCommands(new DriveRobotOrientedOmegaCommand(swerve, -0.20, 0, 0).withTimeout(1));
    addCommands(new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.COMPACT, coral));

    addCommands(new DriveToFieldLocationCommand(swerve, blueLeftOuterStation, 0.05));
    addCommands(new IntakeCoralCommand(coral, false)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, 0.25, 0, blueLeftOuterStation.pose.getRotation().getDegrees() + allianceOffset)));

    // ------------------------the-did-it-work-line------------------------------------ //

    // addCommands(new DriveThroughFieldLocationCommand(swerve, blueLeftPickupTransit, speed));
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
