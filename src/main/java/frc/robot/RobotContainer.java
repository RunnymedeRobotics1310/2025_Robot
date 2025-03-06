// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OiConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.algae.DefaultAlgaeCommand;
import frc.robot.commands.coral.DefaultCoralCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.TeleopDriveCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
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
  private final LimelightVisionSubsystem visionSubsystem =
      new LimelightVisionSubsystem(Constants.VISION_CONFIG);
  private final SwerveSubsystem swerveDriveSubsystem =
      new SwerveSubsystem(Swerve.SUBSYSTEM_CONFIG, visionSubsystem);
  private final CoralSubsystem coralSubsystem = new CoralSubsystem(visionSubsystem);
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

  // Driver and operator controllers
  private final OperatorInput operatorInput =
      new OperatorInput(
          OiConstants.DRIVER_CONTROLLER_PORT,
          OiConstants.OPERATOR_CONTROLLER_PORT,
          OiConstants.CONTROLLER_DEADBAND, swerveDriveSubsystem, coralSubsystem, visionSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Initialize all Subsystem default commands
    swerveDriveSubsystem.setDefaultCommand(
        new TeleopDriveCommand(swerveDriveSubsystem, visionSubsystem, operatorInput));

    coralSubsystem.setDefaultCommand(new DefaultCoralCommand(coralSubsystem, operatorInput));

    algaeSubsystem.setDefaultCommand(new DefaultAlgaeCommand(algaeSubsystem, operatorInput));

    // Configure the button bindings - pass in all subsystems
    operatorInput.configureButtonBindings(
        swerveDriveSubsystem, coralSubsystem, pneumaticsSubsystem, climbSubsystem, algaeSubsystem, visionSubsystem);

    operatorInput.initAutoSelectors();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//  public Command getAutonomousCommand() {
//    // return new Score3L4AutoCommand(swerveDriveSubsystem, 0);
//    //    return new DriveToLeftCenterPointAutoCommand(swerveDriveSubsystem);
//    return new Score1CoralCenterAutoCommand(swerveDriveSubsystem, coralSubsystem, visionSubsystem, 0);
//  }

  public Command getAutonomousCommand() {
    return operatorInput.getAutonomousCommand();
  }


}
