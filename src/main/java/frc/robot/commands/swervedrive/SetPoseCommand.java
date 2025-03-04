package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetPoseCommand extends InstantCommand {

  public SetPoseCommand(SwerveSubsystem driveSubsystem, Pose2d pose) {
    super(
        () -> {
          System.out.println("SetPoseCommand: Set the current pose to " + pose);

          driveSubsystem.resetOdometry(pose);
        });
  }

  @Override
  public boolean runsWhenDisabled() {
    // Allow the gyro heading to be set when the robot is disabled
    return true;
  }
}
