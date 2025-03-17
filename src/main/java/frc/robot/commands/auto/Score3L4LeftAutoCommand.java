package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.FieldLocation.*;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score3L4LeftAutoCommand extends BaseAutoCommand {

  public Score3L4LeftAutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision, double delay) {
    super(swerve, vision, coral);

    addCommands(new SetAllianceGyroCommand(swerve, 180));
    addCommands(new WaitCommand(delay));

    addCommands(goScoreL4Coral(PRE_SCORE_LEFT_4));

    addCommands(autoIntake(blueLeftOuterStation));

    addCommands(goScoreL4Coral(PRE_SCORE_LEFT_2));

    addCommands(autoIntake(blueLeftOuterStation));

    addCommands(goScoreL4Coral(PRE_SCORE_LEFT_3));

    addCommands(autoIntake(blueLeftOuterStation));
  }
}
