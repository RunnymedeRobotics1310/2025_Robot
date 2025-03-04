package frc.robot.commands.swervedrive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class NullDriveCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;

    public NullDriveCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }


    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        swerve.stop();
    }
}
