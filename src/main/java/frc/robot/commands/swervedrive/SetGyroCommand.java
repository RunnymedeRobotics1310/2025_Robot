package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetGyroCommand extends InstantCommand {

  /**
   * Set the current heading in the driveSubsystem
   *
   * @param driveSubsystem
   */
  public SetGyroCommand(SwerveSubsystem driveSubsystem, double yaw) {
    super(
        () -> {
          System.out.println("ZeroGyroCommand: Set the current heading to " + yaw);

          driveSubsystem.setYaw(yaw);
        });
  }

  @Override
  public boolean runsWhenDisabled() {
    // Allow the gyro heading to be set when the robot is disabled
    return true;
  }
}
