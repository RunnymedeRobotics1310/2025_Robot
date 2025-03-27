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
    if (delay > 0) {
      addCommands(new WaitCommand(delay));
    }

    addCommands(scoreL4CoralAndIntake(PRE_SCORE_LEFT_4, blueLeftOuterStation, 3));

    addCommands(scoreL4CoralAndIntake(PRE_SCORE_LEFT_2, blueLeftOuterStation, 1.5));

    //    addCommands(scoreL4CoralAndIntake(PRE_SCORE_LEFT_3, blueLeftOuterStation));

  }
}
