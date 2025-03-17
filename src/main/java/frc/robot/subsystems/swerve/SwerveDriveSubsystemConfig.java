package frc.robot.subsystems.swerve;

import ca.team1310.swerve.core.config.CoreSwerveConfig;

public record SwerveDriveSubsystemConfig(
    boolean enabled,
    CoreSwerveConfig coreConfig,
    SwerveTranslationConfig translationConfig,
    SwerveRotationConfig rotationConfig,
    boolean telemetryEnabled) {}
