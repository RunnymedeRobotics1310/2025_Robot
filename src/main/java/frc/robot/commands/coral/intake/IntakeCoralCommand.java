package frc.robot.commands.coral.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralConstants.ArmAngle;
import frc.robot.Constants.CoralConstants.ElevatorHeight;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.ReverseButAlsoTeleopDriveCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/** Pulls in coral until it is fully inside the arm, then stops the wheels. */
public class IntakeCoralCommand extends LoggingCommand {

  private final CoralSubsystem coralSubsystem;
  private final boolean isFar;
  private final boolean andRun;
  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;
  private final OperatorInput oi;

  public IntakeCoralCommand(CoralSubsystem coralSubsystem, boolean isFar) {

    this.coralSubsystem = coralSubsystem;
    this.isFar = isFar;
    this.andRun = false;
    this.swerve = null;
    this.vision = null;
    this.oi = null;

    addRequirements(coralSubsystem);
  }

  public IntakeCoralCommand(CoralSubsystem coralSubsystem, boolean isFar, boolean andRun, SwerveSubsystem swerve, LimelightVisionSubsystem vision, OperatorInput oi) {
    this.coralSubsystem = coralSubsystem;
    this.isFar = isFar;
    this.andRun = andRun;
    this.swerve = swerve;
    this.vision = vision;
    this.oi = oi;

    addRequirements(coralSubsystem);
  }

  @Override
  public void initialize() {
    logCommandStart();
    coralSubsystem.setIntakeHardLimit(true);
    coralSubsystem.setIntakeSpeed(CoralConstants.CORAL_INTAKE_SPEED);
  }

  @Override
  public void execute() {

    if (isFar) {
      coralSubsystem.moveElevatorToHeight(ElevatorHeight.FAR_INTAKE);
      coralSubsystem.moveArmToAngle(ArmAngle.FAR_INTAKE);
    } else {
      coralSubsystem.moveElevatorToHeight(ElevatorHeight.CLOSE_INTAKE);
      coralSubsystem.moveArmToAngle(ArmAngle.CLOSE_INTAKE);
    }
  }

  @Override
  public boolean isFinished() {

    return coralSubsystem.isCoralDetected();
  }

  @Override
  public void end(boolean interrupted) {
    coralSubsystem.setIntakeSpeed(0);
    coralSubsystem.setIntakeHardLimit(false);
    logCommandEnd(interrupted);


    if (andRun) {
      Pose2d bluePose = swerve.getPose();
      if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
        bluePose = RunnymedeUtils.getRedAlliancePose(bluePose);
      }

      // If in teleop, then automatically schedule the robot move to compact state
      if (DriverStation.isTeleopEnabled()
          && !interrupted
          && bluePose.getX() < 2
          && (bluePose.getY() < 2
              || bluePose.getY() > Constants.FieldConstants.FIELD_EXTENT_METRES_Y - 2)) {
        CommandScheduler.getInstance()
            .schedule(new ReverseButAlsoTeleopDriveCommand(swerve, vision, oi));
      }
    }
  }
}
