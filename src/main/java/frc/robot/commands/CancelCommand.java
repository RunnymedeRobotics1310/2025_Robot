package frc.robot.commands;

import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CancelCommand extends LoggingCommand {

  private final OperatorInput operatorInput;

  private final SwerveSubsystem driveSubsystem;
  private final CoralSubsystem coralSubsystem;

  /**
   * Cancel the commands running on all subsystems.
   *
   * <p>All subsystems must be passed to this command, and each subsystem should have a stop command
   * that safely stops the robot from moving.
   */
  public CancelCommand(
      OperatorInput operatorInput, SwerveSubsystem driveSubsystem, CoralSubsystem coralSubsystem) {

    this.operatorInput = operatorInput;
    this.driveSubsystem = driveSubsystem;
    this.coralSubsystem = coralSubsystem;

    addRequirements(driveSubsystem, coralSubsystem);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    /*
     * The Cancel command is not interruptable and only ends when the cancel button is released.
     */
    return InterruptionBehavior.kCancelIncoming;
  }

  @Override
  public void initialize() {

    logCommandStart();

    stopAll();
  }

  @Override
  public void execute() {
    stopAll();
  }

  @Override
  public boolean isFinished() {

    // The cancel command has a minimum timeout of .5 seconds
    if (!hasElapsed(.5)) {
      return false;
    }

    // Only end once the cancel button is released after .5 seconds has elapsed
    if (!operatorInput.isCancel()) {
      setFinishReason("Cancel button released");
      return true;
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    logCommandEnd(interrupted);
  }

  private void stopAll() {

    // Stop all of the robot movement
    coralSubsystem.stop();
    driveSubsystem.driveFieldOriented(0, 0, 0);
  }
}
