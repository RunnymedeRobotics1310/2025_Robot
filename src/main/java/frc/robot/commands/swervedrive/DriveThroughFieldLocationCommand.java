package frc.robot.commands.swervedrive;

import static frc.robot.Constants.AutoConstants.*;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveThroughFieldLocationCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final FieldLocation location;
    private final double speed;

    public DriveThroughFieldLocationCommand(SwerveSubsystem swerve, FieldLocation location, double speed) {
        this.swerve = swerve;
        this.location = location;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        double xDif = location.pose.getX() - currentPose.getX();
        double yDif = location.pose.getY() - currentPose.getY();
        double targetAngle = location.pose.getRotation().getDegrees();

        double factor = Math.max(xDif, yDif);
        double vX = xDif / factor * speed;
        double vY = yDif / factor * speed;


        swerve.driveFieldOriented(
                vX,
                vY,
                swerve.computeOmega(targetAngle));
    }

    @Override
    public boolean isFinished() {
        return (SwerveUtils.isCloseEnough(
                swerve.getPose().getTranslation(), location.pose.getTranslation(), 0.20));
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}
