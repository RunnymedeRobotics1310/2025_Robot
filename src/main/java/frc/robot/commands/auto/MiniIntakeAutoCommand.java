package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.swervedrive.DriveIntoWallCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class MiniIntakeAutoCommand extends SequentialCommandGroup {

    public MiniIntakeAutoCommand(SwerveSubsystem swerve, CoralSubsystem coral) {
        addCommands(new IntakeCoralCommand(coral, false)
            .deadlineFor(new DriveIntoWallCommand(swerve, 0.25, 0, 0)));
    }
    
}
