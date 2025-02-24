package frc.robot.commands.coral.intake;

import frc.robot.Constants.CoralConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

/** Pushes coral out of the arm, then stops the wheels when the coral is gone. */
public class PlantCoralCommand extends LoggingCommand {

  private final CoralSubsystem coralSubsystem;
  private double intakeStartPos = 0;

  /**
   * Plant coral runs the intake motor for a set number of rotations. Use it for autos.
   *
   * @param coralSubsystem
   */
  public PlantCoralCommand(CoralSubsystem coralSubsystem) {
    this.coralSubsystem = coralSubsystem;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    intakeStartPos = coralSubsystem.getIntakeEncoder();
    logCommandStart();
  }

  @Override
  public void execute() {
    coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_OUTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {
    // ends the command after spinning the intake motor PLANT_ROTATIONS times.

    if (Math.abs(intakeStartPos - coralSubsystem.getIntakeEncoder())
            <= CoralConstants.PLANT_ROTATIONS
        && !coralSubsystem.isCoralDetected()) {
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
