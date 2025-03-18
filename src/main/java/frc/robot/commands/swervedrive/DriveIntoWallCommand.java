package frc.robot.commands.swervedrive;

import ca.team1310.swerve.utils.SwerveUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveIntoWallCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final double vX;
    private final double vY;
    private final double headingDeg;

    public DriveIntoWallCommand(SwerveSubsystem swerve, double vX, double vY, double headingDeg) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingDeg = SwerveUtils.normalizeDegrees(headingDeg);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {
        swerve.driveRobotOriented(vX, vY, swerve.computeOmega(headingDeg));
    }

    @Override
    public boolean isFinished() {
        return swerve.getUltrasonicDistanceM() <= 0.03;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        swerve.stop();
    }

}
