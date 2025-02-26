package frc.robot.commands.pneumatics;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ToggleCompressorCommand extends LoggingCommand {

    private final PneumaticsSubsystem pneumaticsSubsystem;

    public ToggleCompressorCommand(PneumaticsSubsystem pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {
        if (pneumaticsSubsystem.getCompressorEnabled()) {
            pneumaticsSubsystem.setCompresor(false);
        } else {
            pneumaticsSubsystem.setCompresor(true);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
