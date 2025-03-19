package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class AutoClimbCommand extends LoggingCommand {

  private final ClimbSubsystem climbSubsystem;
  Timer climbTimer = new Timer();

  public AutoClimbCommand(ClimbSubsystem climbSubsystem) {

    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
    climbTimer.stop();
    climbTimer.reset();
  }

  @Override
  public void execute() {

    if (climbSubsystem.isCageInPosition() && !climbTimer.isRunning()) {

      climbTimer.start();

    } else if (!climbSubsystem.isCageInPosition() && climbTimer.isRunning()) {

      climbTimer.stop();
      climbTimer.reset();

    } else if (climbTimer.hasElapsed(0.3)) {

      climbSubsystem.setClimbDeployed(true);
    }
  }

  @Override
  public boolean isFinished() {
    return climbSubsystem.isCageInPosition() && climbTimer.hasElapsed(0.3);
  }

  @Override
  public void end(boolean interrupted) {
    climbTimer.stop();
    climbTimer.reset();
    logCommandEnd(interrupted);
  }
}
