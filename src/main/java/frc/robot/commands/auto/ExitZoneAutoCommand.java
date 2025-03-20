package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedOmegaCommand;
import frc.robot.commands.swervedrive.SetAllianceGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ExitZoneAutoCommand extends SequentialCommandGroup {

  public ExitZoneAutoCommand(SwerveSubsystem swerve, double delay) {
    addCommands(new WaitCommand(delay));

    double allianceOffset = 0;
    if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
      allianceOffset = 180;
    }

    addCommands(new SetAllianceGyroCommand(swerve, 180));

    addCommands(new DriveRobotOrientedCommand(swerve, 0.50, 0.00, 180 + allianceOffset)
            .withTimeout(0.55));
  }
}
