package frc.robot.commands.auto;

import static frc.robot.Constants.AutoConstants.FieldLocation.*;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class Score3L4RightAutoCommand extends BaseAutoCommand {

  public Score3L4RightAutoCommand(
      SwerveSubsystem swerve, CoralSubsystem coral, LimelightVisionSubsystem vision, double delay) {
    super(swerve, vision, coral);

    addCommands(new SetAllianceGyroCommand(swerve, 180));
    addCommands(new WaitCommand(delay));

    goScoreL4Coral(PRE_SCORE_RIGHT_4);

    autoIntake(blueRightOuterStation);

    goScoreL4Coral(PRE_SCORE_RIGHT_2);

    autoIntake(blueRightOuterStation);

    goScoreL4Coral(PRE_SCORE_RIGHT_3);

    autoIntake(blueRightOuterStation);
  }
}
