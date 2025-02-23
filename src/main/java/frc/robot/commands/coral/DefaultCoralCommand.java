package frc.robot.commands.coral;

import frc.robot.Constants.CoralConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.CoralSubsystem;

public class DefaultCoralCommand extends LoggingCommand {

    private final CoralSubsystem coralSubsystem;
    private final OperatorInput  operatorInput;

    public DefaultCoralCommand(CoralSubsystem coralSubsystem, OperatorInput operatorInput) {
        this.coralSubsystem = coralSubsystem;
        this.operatorInput  = operatorInput;

        addRequirements(coralSubsystem);
    }


    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {


        double  elevatorInput = operatorInput.getElevatorInput();
        boolean ejectButton   = operatorInput.getEjectButton();
        boolean injectButton  = operatorInput.getInjectButton();

        // Elevator commands
        coralSubsystem.setElevatorSpeed(elevatorInput * CoralConstants.ELEVATOR_TUNE_MAX_SPEED);

        double armStick = operatorInput.getArmStick();
        if (Math.abs(armStick) > 0) {
            coralSubsystem.setArmSpeed(armStick * CoralConstants.ARM_TUNE_MAX_SPEED);
        }
        else {
            coralSubsystem.setArmSpeed(0);
        }

        // Intake commands
        if (ejectButton) {
            coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_OUTAKE_SPEED);
        }
        else if (injectButton) {
            // Intake & outtake are in the same direction
            coralSubsystem.setIntakeSpeed(-CoralConstants.CORAL_INTAKE_SPEED);
        }
        else {
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