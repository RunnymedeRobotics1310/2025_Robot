package frc.robot.commands.swervedrive;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.AutoConstants.FieldLocation;

public class DriveToFieldLocationCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final FieldLocation location;
    private final double targetHeadingDeg;

    public DriveToFieldLocationCommand(SwerveSubsystem swerve, FieldLocation location) {
        this.swerve = swerve;
        this.location = location;
        this.targetHeadingDeg = SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees());
    }

    public DriveToFieldLocationCommand(
            SwerveSubsystem swerve, FieldLocation redLocation, FieldLocation blueLocation) {
        this.swerve = swerve;
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            this.location = blueLocation;
        } else {
            this.location = redLocation;
        }
        this.targetHeadingDeg = SwerveUtils.normalizeDegrees(location.pose.getRotation().getDegrees());
    }

    @Override
    public void initialize() {
        logCommandStart();
        log("Pose: " + swerve.getPose());
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        double xDif = location.pose.getX() - currentPose.getX();
        double yDif = location.pose.getY() - currentPose.getY();

        double angleDif = SwerveUtils.normalizeDegrees(targetHeadingDeg - currentPose.getRotation().getDegrees());
//        log("Xdif: " + xDif + " Ydif: " + yDif + " ºdif: " + angleDif);

        swerve.driveFieldOriented(
                swerve.computeTranslateVelocity(xDif, 0.02),
                swerve.computeTranslateVelocity(yDif, 0.02),
                swerve.computeOmega(targetHeadingDeg));
    }

    @Override
    public boolean isFinished() {
//        return (SwerveUtils.isCloseEnough(
//                swerve.getPose().getTranslation(), location.pose.getTranslation(), 0.05)
//                && SwerveUtils.isCloseEnough(swerve.getPose().getRotation().getDegrees(), targetHeadingDeg, 10));
        boolean done = (SwerveUtils.isCloseEnough(
                swerve.getPose().getTranslation(), location.pose.getTranslation(), 0.05)
                && SwerveUtils.isCloseEnough(swerve.getYaw(), targetHeadingDeg, 10));
    if (done) {
      System.out.println(
          "REACHED DESTINATION: x["
              + swerve.getPose().getX()
              + "] y["
              + swerve.getPose().getY()
              + "], deg["
              + swerve.getPose().getRotation().getDegrees()
              + "]");
    }
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        swerve.stop();
    }
}
