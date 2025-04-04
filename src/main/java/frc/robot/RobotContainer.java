// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OiConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.coral.DefaultCoralCommand;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.DriveToReefTagCommand;
import frc.robot.commands.swervedrive.NullDriveCommand;
import frc.robot.commands.swervedrive.TeleopDriveCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem(Swerve.SUBSYSTEM_CONFIG);
  private final LimelightVisionSubsystem visionSubsystem =
      new LimelightVisionSubsystem(Constants.VisionConstants.VISION_CONFIG, swerveDriveSubsystem);
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  private final LightingSubsystem lightingSubsystem = new LightingSubsystem(swerveDriveSubsystem);
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  // Driver and operator controllers
  private final OperatorInput operatorInput =
      new OperatorInput(
          OiConstants.DRIVER_CONTROLLER_PORT,
          OiConstants.OPERATOR_CONTROLLER_PORT,
          OiConstants.CONTROLLER_DEADBAND,
          swerveDriveSubsystem,
          coralSubsystem,
          visionSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Initialize all Subsystem default commands
    swerveDriveSubsystem.setDefaultCommand(
        new TeleopDriveCommand(swerveDriveSubsystem, visionSubsystem, operatorInput));

    coralSubsystem.setDefaultCommand(new DefaultCoralCommand(coralSubsystem, operatorInput));

    // Configure the button bindings - pass in all subsystems
    operatorInput.configureButtonBindings(
        swerveDriveSubsystem, coralSubsystem, pneumaticsSubsystem, climbSubsystem, visionSubsystem);
    operatorInput.configureDashboardBindings(
        swerveDriveSubsystem, coralSubsystem, pneumaticsSubsystem, climbSubsystem, visionSubsystem);

    operatorInput.initAutoSelectors();

    initNamedCommands();
  }

  private void initNamedCommands() {
    NamedCommands.registerCommand("intake-close", new IntakeCoralCommand(coralSubsystem, false));
    NamedCommands.registerCommand("intake-far", new IntakeCoralCommand(coralSubsystem, false));
    NamedCommands.registerCommand(
        "coral-compact",
        new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.COMPACT, coralSubsystem));
    NamedCommands.registerCommand(
        "coral-l1",
        new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L1, coralSubsystem));
    NamedCommands.registerCommand(
        "coral-l2",
        new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L2, coralSubsystem));
    NamedCommands.registerCommand(
        "coral-l3",
        new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L3, coralSubsystem));
    NamedCommands.registerCommand(
        "coral-l4",
        new MoveToCoralPoseCommand(Constants.CoralConstants.CoralPose.SCORE_L4, coralSubsystem));
    NamedCommands.registerCommand("plant", new PlantCoralCommand(coralSubsystem));
    NamedCommands.registerCommand(
        "drivetotag-l1",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_1));
    NamedCommands.registerCommand(
        "drivetotag-l2",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_2));
    NamedCommands.registerCommand(
        "drivetotag-l3",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_3));
    NamedCommands.registerCommand(
        "drivetotag-l4",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_4));
    NamedCommands.registerCommand(
        "drivetotag-l5",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_5));
    NamedCommands.registerCommand(
        "drivetotag-l6",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_6));
    NamedCommands.registerCommand(
        "drivetotag-r1",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_1));
    NamedCommands.registerCommand(
        "drivetotag-r2",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_2));
    NamedCommands.registerCommand(
        "drivetotag-r3",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_3));
    NamedCommands.registerCommand(
        "drivetotag-r4",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_4));
    NamedCommands.registerCommand(
        "drivetotag-r5",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_5));
    NamedCommands.registerCommand(
        "drivetotag-r6",
        new DriveToReefTagCommand(
            swerveDriveSubsystem,
            visionSubsystem,
            Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_6));
    NamedCommands.registerCommand(
        "stop", new NullDriveCommand(swerveDriveSubsystem).withTimeout(0.1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //  public Command getAutonomousCommand() {
  //    // return new Score3L4AutoCommand(swerveDriveSubsystem, 0);
  //    //    return new DriveToLeftCenterPointAutoCommand(swerveDriveSubsystem);
  //    return new Score1CoralCenterAutoCommand(swerveDriveSubsystem, coralSubsystem,
  // visionSubsystem, 0);
  //  }

  public Command getAutonomousCommand() {
    return operatorInput.getAutonomousCommand();
  }
}
