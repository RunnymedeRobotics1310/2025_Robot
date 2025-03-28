package frc.robot.commands.coral.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CoralConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

/** Pushes coral out of the arm, then stops the wheels when the coral is gone. */
public class PlantCoralCommand extends LoggingCommand {

  private final CoralSubsystem coralSubsystem;
  private double intakeStartPos = 0;
  private boolean hasShot = false;
  private double shotTime = 0;

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
    hasShot = false;
    shotTime = 0;
    logCommandStart();
  }

  @Override
  public void execute() {
    coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_OUTAKE_SPEED);

    if (!coralSubsystem.isCoralDetected() && !hasShot){
      shotTime = Timer.getFPGATimestamp();
      hasShot = true;
    }
  }

  @Override
  public boolean isFinished() {
    // ends the command after spinning the intake motor PLANT_ROTATIONS times and after waiting 300 MS

    if (Math.abs(intakeStartPos - coralSubsystem.getIntakeEncoder())
            <= CoralConstants.PLANT_ROTATIONS
        && !coralSubsystem.isCoralDetected() && (Timer.getFPGATimestamp() - shotTime) > CoralConstants.POST_SHOT_WAIT_TIME){
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
