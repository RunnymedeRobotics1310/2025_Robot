package frc.robot.commands;

import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CancelCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;

    public CancelCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        swerve.driveFieldOriented(0, 0, 0);
    }
}
