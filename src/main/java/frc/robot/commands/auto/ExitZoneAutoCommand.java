package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.SetGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

  public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {
    double headingOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      headingOffset = 180;
    }
    addCommands(new WaitCommand(delay));

    addCommands(new SetGyroCommand(swerve, 180 + headingOffset));

    addCommands(
        new WaitCommand(1)
            .deadlineFor(new DriveRobotOrientedCommand(swerve, 2, 0, 180 + headingOffset)));
  }
}
