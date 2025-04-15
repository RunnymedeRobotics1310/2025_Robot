package frc.robot.commands.coral;

import frc.robot.Constants.CoralConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.CoralSubsystem;

public class DefaultCoralCommand extends LoggingCommand {

  private final CoralSubsystem coralSubsystem;
  private final OperatorInput operatorInput;

  public DefaultCoralCommand(CoralSubsystem coralSubsystem, OperatorInput operatorInput) {
    this.coralSubsystem = coralSubsystem;
    this.operatorInput = operatorInput;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
  }

  @Override
  public void execute() {

    double elevatorInput = operatorInput.getElevatorInput();
    double armInput = operatorInput.getArmStick();
    boolean ejectButton = operatorInput.getEjectButton();
    boolean injectButton = operatorInput.getInjectButton();
    boolean plant = operatorInput.getPlant();

    // Elevator commands
    if (elevatorInput == 0) {
      if (!coralSubsystem.isElevatorAtLowerLimit()) {
        coralSubsystem.setElevatorSpeed(CoralConstants.ELEVATOR_HOLD_STEADY_SPEED);
      }
    } else {
      coralSubsystem.setElevatorSpeed(elevatorInput * CoralConstants.ELEVATOR_TUNE_MAX_SPEED);
    }

    coralSubsystem.setArmSpeed(armInput * CoralConstants.ARM_TUNE_MAX_SPEED);

    // Intake commands
    if (plant) {
      if (coralSubsystem.getArmAngle() <= 70) {
        coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_OUTAKE_SLOW_SPEED);
      } else {
        coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_OUTAKE_SPEED);
      }
    } else if (ejectButton) {
      coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_EJECT_SPEED);
    } else if (injectButton) {
      // Intake & outtake are in the same direction
      coralSubsystem.setIntakeSpeed(-CoralConstants.CORAL_EJECT_SPEED);
    } else {
      coralSubsystem.setIntakeSpeed(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    coralSubsystem.stop();
  }
}
