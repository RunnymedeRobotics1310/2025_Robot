package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Score1CoralCenterAutoCommand extends SequentialCommandGroup {

    public Score1CoralCenterAutoCommand(SwerveSubsystem swerve, double delay) {

        addCommands(new WaitCommand(delay));

        addCommands(new DriveToFieldLocationCommand(swerve, Constants.AutoConstants.FieldLocation.preScoreBlueLeft6));
//        addCommands(new SetupScoreCommand(swerve, coral, l4));
//        addCommands(new PlantCoralCommand(coral));
//        addCommands(new SetCoralPoseCommand(coral, compact));


    }

}
