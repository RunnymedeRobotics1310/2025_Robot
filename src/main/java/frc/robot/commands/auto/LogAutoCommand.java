package frc.robot.commands.auto;

import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LogAutoCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final CoralSubsystem coral;

  public LogAutoCommand(SwerveSubsystem swerve) {
    this(swerve, null);
  }

  public LogAutoCommand(SwerveSubsystem swerve, CoralSubsystem coral) {
    this.swerve = swerve;
    this.coral = coral;
  }

  public void initialize() {

    StringBuilder sb = new StringBuilder();
    sb.append("Alliance: ")
        .append(RunnymedeUtils.getRunnymedeAlliance())
        .append(" Has Vis Pose: ")
        .append(swerve.hasVisPose())
        .append(" Pose: ")
        .append(swerve)
        .append(" Coral: ")
        .append(coral);
    log(sb.toString());
  }
}
