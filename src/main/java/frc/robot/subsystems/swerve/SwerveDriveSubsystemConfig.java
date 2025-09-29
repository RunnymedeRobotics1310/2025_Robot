package frc.robot.subsystems.swerve;

import ca.team1310.swerve.core.config.CoreSwerveConfig;
import ca.team1310.swerve.gyro.config.GyroConfig;
import ca.team1310.swerve.vision.config.LimelightConfig;

public record SwerveDriveSubsystemConfig(
    boolean enabled,
    CoreSwerveConfig coreConfig,
    GyroConfig gyroConfig,
    LimelightConfig limelightConfig,
    SwerveTranslationConfig translationConfig,
    SwerveRotationConfig rotationConfig,
    boolean telemetryEnabled) {}
