package frc.robot.commands.coral.intake;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.ArmAngle;
import frc.robot.Constants.CoralConstants.ElevatorHeight;
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
    coralSubsystem.moveElevatorToHeight(ElevatorHeight.INTAKE);
    coralSubsystem.moveArmToAngle(ArmAngle.INTAKE);
  }

  @Override
  public boolean isFinished() {

    
    return coralSubsystem.isCoralDetected();
  }

  @Override
  public void end(boolean interrupted) {
    coralSubsystem.setIntakeSpeed(0);
    logCommandEnd(interrupted);
  }
}
