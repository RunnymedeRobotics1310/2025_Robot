package frc.robot.commands.swervedrive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Lock the robot pose as long as this command is active (i.e. as long as the operator is pressing
 * the button.
 */
public class LockCommand extends LoggingCommand {

  private final SwerveSubsystem swerve;

  public LockCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
    swerve.lock();
  }
}
