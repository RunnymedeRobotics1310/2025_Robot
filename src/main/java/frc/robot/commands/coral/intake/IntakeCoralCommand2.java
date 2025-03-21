package frc.robot.commands.coral.intake;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.ArmAngle;
import frc.robot.Constants.CoralConstants.ElevatorHeight;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

/** Pulls in coral until it is fully inside the arm, then stops the wheels. */
public class IntakeCoralCommand2 extends LoggingCommand {

  private final CoralSubsystem coralSubsystem;
  private final boolean isFar;

  public IntakeCoralCommand2(CoralSubsystem coralSubsystem, boolean isFar) {

    this.coralSubsystem = coralSubsystem;
    this.isFar = isFar;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
    
    // Switch the intake to a hard limit
    coralSubsystem.setIntakeHardLimit(true);
    coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if (isFar) {
      coralSubsystem.moveElevatorToHeight(ElevatorHeight.FAR_INTAKE);
    } else {
      coralSubsystem.moveElevatorToHeight(ElevatorHeight.CLOSE_INTAKE);
    }

    coralSubsystem.moveArmToAngle(ArmAngle.INTAKE);
  }

  @Override
  public boolean isFinished() {
    
    return coralSubsystem.isCoralDetected();
  }

  @Override
  public void end(boolean interrupted) {
    coralSubsystem.startCoralAlignment();
    logCommandEnd(interrupted);
  }
}
