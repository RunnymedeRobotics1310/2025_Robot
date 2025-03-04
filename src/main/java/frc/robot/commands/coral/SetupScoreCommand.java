package frc.robot.commands.coral;

import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetupScoreCommand extends LoggingCommand {

    /*
    Get distance with ultrasonic sensor
    Move toward desired distance
    While moving move the elevator and arm to position
    */
    private final CoralSubsystem coralSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final Constants.CoralConstants.CoralPose coralPose;
    private final Constants.CoralConstants.DesiredDistanceToTargetCM distanceToTarget;

    boolean atElevatorHeight = false;
    boolean atArmAngle = false;
    boolean atCorrectDistance = false;

    private double deltaDistance;
    private double currentDistance;

    public SetupScoreCommand(Constants.CoralConstants.CoralPose coralPose, Constants.CoralConstants.DesiredDistanceToTargetCM desiredDistanceToTarget, CoralSubsystem coralSubsystem, SwerveSubsystem swerveSubsystem) {

        this.coralSubsystem = coralSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.coralPose = coralPose;
        this.distanceToTarget = desiredDistanceToTarget;

        addRequirements(coralSubsystem);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart();

        atElevatorHeight = false;
        atArmAngle = false;
        atCorrectDistance = false;
    }

    @Override
    public void execute() {
        atElevatorHeight = coralSubsystem.moveElevatorToHeight(coralPose.elevatorHeight);
        atArmAngle = coralSubsystem.moveArmToAngle(coralPose.armAngle);

        currentDistance = coralSubsystem.getUltrasonicDistanceCm();
        deltaDistance = currentDistance - distanceToTarget.getDistance();



        if (Math.abs(deltaDistance) < Constants.CoralConstants.SCORING_DISTANCE_TOLERANCE) {
            atCorrectDistance = true;
            swerveSubsystem.stop();
        } else {
            swerveSubsystem.driveRobotOriented(0.25 * Math.signum(deltaDistance),0,0);
            atCorrectDistance = false;
        }
    }

    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        swerveSubsystem.stop();
        coralSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if (!coralSubsystem.isCoralDetected()){
            return true;
        }
        if (atElevatorHeight && atArmAngle && atCorrectDistance) {
            return true;
        }
        return false;
    }
}





