package frc.robot.commands.coral.arm;

import frc.robot.Constants.CoralConstants.ArmAngle;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class MoveArmToAngleCommand extends LoggingCommand {

    private final CoralSubsystem coralSubsystem;

    private final ArmAngle       armAngle;

    private boolean              atAngle = false;

    public MoveArmToAngleCommand(ArmAngle armAngle, CoralSubsystem coralSubsystem) {

        this.coralSubsystem = coralSubsystem;
        this.armAngle       = armAngle;

        addRequirements(coralSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {

        atAngle = coralSubsystem.moveArmToAngle(armAngle);

    }

    @Override
    public boolean isFinished() {

        // Check if the arm is at the requested position.
        if (atAngle) {
            return true;
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}
