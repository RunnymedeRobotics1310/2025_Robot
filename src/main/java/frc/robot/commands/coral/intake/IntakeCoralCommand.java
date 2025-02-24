package frc.robot.commands.coral.intake;

import frc.robot.Constants.CoralConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

/** Pulls in coral until it is fully inside the arm, then stops the wheels. */
public class IntakeCoralCommand extends LoggingCommand {

  private final CoralSubsystem coralSubsystem;
  private boolean firstDetect = true;
  private double encoderOnFirstDetect;

  public IntakeCoralCommand(CoralSubsystem coralSubsystem) {

    this.coralSubsystem = coralSubsystem;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {
    coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_INTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {

    if (coralSubsystem.isCoralDetected() && firstDetect) {
      // stop the motors when coral is detected
      encoderOnFirstDetect = coralSubsystem.getIntakeEncoder();
    }

    if (!firstDetect
        && (Math.abs(encoderOnFirstDetect - coralSubsystem.getIntakeEncoder()))
            <= CoralConstants.INTAKE_ROTATIONS) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    coralSubsystem.setIntakeSpeed(0);
    logCommandEnd(interrupted);
  }
}
