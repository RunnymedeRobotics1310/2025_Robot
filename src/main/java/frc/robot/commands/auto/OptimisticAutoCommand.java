package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.FieldLocation.*;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.SetPoseCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class OptimisticAutoCommand extends SequentialCommandGroup {

  public OptimisticAutoCommand(SwerveSubsystem swerve, double delay) {
    super();

    addCommands(new SetPoseCommand(swerve, new Pose2d(7.6, 2.00, Rotation2d.fromDegrees(0))));
    addCommands(new ZeroGyroCommand(swerve));

    addCommands(new WaitCommand(delay));

    addCommands(new DriveToFieldLocationCommand(swerve, redE, blueE));
    // Score L4 - 1 - E
    addCommands(new DriveToFieldLocationCommand(swerve, redRightExitTransit, blueRightExitTransit));
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightOuterStation, blueRightOuterStation));
    // Intake Coral
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightPickupTransit, blueRightPickupTransit));
    addCommands(new DriveToFieldLocationCommand(swerve, redB, blueB));

    // Score L4 - 2 - B
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightPickupTransit, blueRightPickupTransit));
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightOuterStation, blueRightOuterStation));
    // Intake Coral
    addCommands(new DriveToFieldLocationCommand(swerve, redC, blueC));

    // Score L4 - 3 - C
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightOuterStation, blueRightOuterStation));
    // Intake Coral
    addCommands(new DriveToFieldLocationCommand(swerve, redD, blueD));

    // Score L4 - 4 - D

    // ------------------ IF TIME ------------------ //

    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightOuterStation, blueRightOuterStation));
    // Intake Coral
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightPickupTransit, blueRightPickupTransit));
    addCommands(new DriveToFieldLocationCommand(swerve, redA, blueA));

    // Score L4 - 5? - A
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightPickupTransit, blueRightPickupTransit));
    addCommands(
        new DriveToFieldLocationCommand(swerve, redRightOuterStation, blueRightOuterStation));
    // Intake Coral

  }
}
