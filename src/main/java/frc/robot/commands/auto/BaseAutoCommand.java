package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.AutoConstants.*;

import static frc.robot.Constants.CoralConstants.CoralPose.*;

import static frc.robot.Constants.CoralConstants.*;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.swervedrive.DriveRobotOrientedCommand;
import frc.robot.commands.swervedrive.DriveThroughFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToFieldLocationCommand;
import frc.robot.commands.swervedrive.DriveToVisibleTagCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class BaseAutoCommand extends SequentialCommandGroup {

    private final SwerveSubsystem swerve;
    private final LimelightVisionSubsystem vision;
    private final CoralSubsystem coral;
    private double allianceOffset = 0;

    public BaseAutoCommand(SwerveSubsystem swerve, LimelightVisionSubsystem vision, CoralSubsystem coral) {
        this.swerve = swerve;
        this.vision = vision;
        this.coral = coral;
        if (RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Red) {
            allianceOffset = 180;
        }
    }

    protected Command driveToLocation(FieldLocation location) {
        return new DriveToFieldLocationCommand(swerve, location);
    }

    protected Command driveThroughLocation(FieldLocation location, double speed) {
        return new DriveThroughFieldLocationCommand(swerve, location, speed);
    }

    protected Command setCoralPose(CoralPose pose) {
        return new MoveToCoralPoseCommand(pose, coral);
    }

    protected Command plant() {
        return new PlantCoralCommand(coral);
    }

    protected Command approachReef(boolean isLeftBranch) {
        return new DriveToVisibleTagCommand(swerve, vision, isLeftBranch)
                .alongWith(setCoralPose(SCORE_L4));
    }



    public Command goScoreL4Coral(FieldLocation location) {
        double locationHeading = location.pose.getRotation().getDegrees() + allianceOffset;

        return driveThroughLocation(location, 2)
        .andThen(approachReef(location.isLeftSide))
        .andThen(plant())
        .andThen(new DriveRobotOrientedCommand(swerve, -0.2, 0, locationHeading).withTimeout(1))
        .andThen(setCoralPose(COMPACT));
    }

    public Command autoIntake(FieldLocation location) {
        double locationHeading = location.pose.getRotation().getDegrees() + allianceOffset;

        return driveThroughLocation(location, 2)
                .andThen(new IntakeCoralCommand(coral, false)
                .deadlineFor(new DriveRobotOrientedCommand(swerve, 0.25, 0, locationHeading)));
    }

}
