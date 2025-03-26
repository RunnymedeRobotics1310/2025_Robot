package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class AutoClimbCommand extends LoggingCommand {

  private final ClimbSubsystem climbSubsystem;
  Timer climbTimer = new Timer();
  private boolean isClimbTriggered;

  public AutoClimbCommand(ClimbSubsystem climbSubsystem) {

    this.climbSubsystem = climbSubsystem;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
    climbTimer.stop();
    climbTimer.reset();
    isClimbTriggered = false;
  }

  @Override
  public void execute() {

    if (climbSubsystem.isCageInPosition() && !climbTimer.isRunning()) {

      climbTimer.start();

    } else if (!climbSubsystem.isCageInPosition() && climbTimer.isRunning()) {

      climbTimer.stop();
      climbTimer.reset();

    } else if (climbTimer.hasElapsed(0.3) || RunnymedeUtils.teleopMatchTimeRemaining() < 0.1) {

      climbSubsystem.setClimbDeployed(true);
    }

    isClimbTriggered = climbSubsystem.isCageInPosition() && climbTimer.hasElapsed(0.3);
  }

  @Override
  public boolean isFinished() {
    return isClimbTriggered;
  }

  @Override
  public void end(boolean interrupted) {
    climbTimer.stop();
    climbTimer.reset();
    logCommandEnd(interrupted);
  }
}
