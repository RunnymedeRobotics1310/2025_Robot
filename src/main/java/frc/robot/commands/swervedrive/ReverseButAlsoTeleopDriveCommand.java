package frc.robot.commands.swervedrive;

import ca.team1310.swerve.utils.SwerveUtils;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class ReverseButAlsoTeleopDriveCommand extends TeleopDriveCommand{
    private final OperatorInput oi;
    private final SwerveSubsystem swerve;
    private double startHeading;
    private double targetAngle;
    private boolean isDriving;
    private Timer timer = new Timer();

    public ReverseButAlsoTeleopDriveCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision, OperatorInput oi) {

        super(swerve, vision, oi);
    this.oi = oi;
    this.swerve = swerve;

    }

    @Override
    public void initialize() {
        super.initialize();
        this.startHeading = swerve.getYaw();
        this.targetAngle = SwerveUtils.normalizeDegrees(startHeading + 180);
        timer.start();

    }

    @Override
    public void execute() {

        // true if any Teleop drive buttons are being used
        isDriving = (
                oi.getDriverControllerAxis(OperatorInput.Stick.LEFT, OperatorInput.Axis.Y) != 0
                || oi.getDriverControllerAxis(OperatorInput.Stick.LEFT, OperatorInput.Axis.X) != 0
                || oi.getDriverControllerAxis(OperatorInput.Stick.RIGHT, OperatorInput.Axis.X) != 0
                || oi.isFaceReef()
                || oi.getRotate180Val()
                || oi.isAlignRightStation()
                || oi.isAlignLeftStation()
                || oi.isSlowMode()
                || oi.isFastMode()
        );

    if (isDriving) super.execute();
    else {
        double reefAngle = swerve.getClosestReefAngle(swerve.getPose().getX(), swerve.getPose().getY());
        swerve.driveFieldOriented(-Math.cos(reefAngle)/2, -Math.sin(reefAngle), swerve.computeOmega(reefAngle));
    }
  }

  public boolean isFinished() {
        return isDriving;
    }
}