package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

  public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {
    addCommands(new WaitCommand(delay));
    addCommands(new WaitCommand(1).deadlineFor(new DriveRobotOrientedCommand(swerve, 2, 0, 180)));

  }
}
