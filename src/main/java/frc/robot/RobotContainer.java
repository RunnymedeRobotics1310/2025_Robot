// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OiConstants;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.swervedrive.TeleopDriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final LimelightVisionSubsystem visionSubsystem = new LimelightVisionSubsystem(Constants.VISION_CONFIG);
    private final SwerveSubsystem swerveDriveSubsystem = new SwerveSubsystem(Constants.Swerve.SUBSYSTEM_CONFIG, visionSubsystem);
    private final OperatorInput operatorInput = new OperatorInput(OiConstants.DRIVER_CONTROLLER_PORT, OiConstants.OPERATOR_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Initialize all Subsystem default commands
        swerveDriveSubsystem.setDefaultCommand(new TeleopDriveCommand(swerveDriveSubsystem, operatorInput));
        // Configure the trigger bindings
        operatorInput.configureBindings(swerveDriveSubsystem);
    }
}
