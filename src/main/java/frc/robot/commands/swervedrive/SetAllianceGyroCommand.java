package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RunnymedeUtils;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetAllianceGyroCommand extends InstantCommand {

  /**
   * Set the current heading in the driveSubsystem
   *
   * @param driveSubsystem
   */
  public SetAllianceGyroCommand(SwerveSubsystem driveSubsystem, double yaw) {
    super(
        () -> {
          double headingOffset = 0;
          if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            headingOffset = 180;
          }

          driveSubsystem.setYaw(yaw + headingOffset);
        });
  }

  @Override
  public boolean runsWhenDisabled() {
    // Allow the gyro heading to be set when the robot is disabled
    return true;
  }
}
