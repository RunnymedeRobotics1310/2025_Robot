package frc.robot.commands.algae;


import frc.robot.Constants.AlgaeConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.AlgaeSubsystem;


public class DefaultAlgaeCommand extends LoggingCommand {

    private final AlgaeSubsystem algaeSubsystem;
    private final OperatorInput operatorInput;
    public boolean algaeIntakeState = false;
    public boolean algaeOuttakeState = false;

    public double speed = 0;

    public DefaultAlgaeCommand(AlgaeSubsystem algaeSubsystem, OperatorInput operatorInput) {

        this.algaeSubsystem = algaeSubsystem;
        this.operatorInput = operatorInput;

        addRequirements(algaeSubsystem);

    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {
        if (operatorInput.getIntakeAlgae() && !algaeOuttakeState) {
            algaeIntakeState = true;
        } else {
            algaeIntakeState = false;
        }

        if (operatorInput.getOuttakeAlgae() && !algaeIntakeState) {
            algaeOuttakeState = true;
        } else {
            algaeOuttakeState = false;
        }

        if (algaeIntakeState) {
            speed = AlgaeConstants.ALGAE_INTAKE_SPEED;
            algaeSubsystem.deployArm(true);
        } else if (algaeOuttakeState) {
            speed = AlgaeConstants.ALGAE_OUTTAKE_SPEED;
            algaeSubsystem.deployArm(false);
        } else {
            speed = 0;
        }

        algaeSubsystem.setIntakeSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        algaeSubsystem.stop();

    }
}


