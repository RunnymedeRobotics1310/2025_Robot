package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.*;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score1CoralCenterAutoCommand extends BaseAutoCommand {

  public Score1CoralCenterAutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision, double delay) {
    super(swerve, vision, coral);

    addCommands(new SetAllianceGyroCommand(swerve, 180));
    addCommands(new WaitCommand(delay));

    goScoreL4Coral(FieldLocation.PRE_SCORE_LEFT_6);
  }
}
