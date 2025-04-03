package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LogAutoCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;
  private final CoralSubsystem coral;

  public LogAutoCommand(SwerveSubsystem swerve, CoralSubsystem coral) {
    this.swerve = swerve;
    this.coral = coral;
  }

  public LogAutoCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.coral = null;
  }

  public void initialize() {

    boolean hasVisPose = SmartDashboard.getBoolean("swerve/hasVisPose", false);
    StringBuilder sb = new StringBuilder();
    sb.append("Alliance: ")
        .append(RunnymedeUtils.getRunnymedeAlliance())
        .append(" Has Vis Pose: ")
        .append(hasVisPose)
        .append(" Pose: ")
        .append(swerve)
        .append(" Coral: ")
        .append(coral);
    log(sb.toString());
  }
}
