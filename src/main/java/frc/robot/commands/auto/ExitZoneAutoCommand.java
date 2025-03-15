package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedOmegaCommand;
import frc.robot.commands.swervedrive.SetAllianceGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

  public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {
    addCommands(new WaitCommand(delay));

    addCommands(new SetAllianceGyroCommand(swerve, 180));

    addCommands(new DriveRobotOrientedOmegaCommand(swerve, 1.00, 0.00, 0).withTimeout(1));
  }
}
