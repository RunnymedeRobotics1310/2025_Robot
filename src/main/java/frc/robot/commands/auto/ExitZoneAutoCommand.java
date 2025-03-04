package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

  public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {
    addCommands(new WaitCommand(delay));
    //        addCommands(new DriveRobotOrientedCommand(swerve, new Pose2d(0, 1), new
    // Rotation2d()));
  }
}
