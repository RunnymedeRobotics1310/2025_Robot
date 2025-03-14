package frc.robot.subsystems.swerve;

public record SwerveRotationConfig(
    double maxRotVelocityRadPS,
    double maxAccelerationRadPS2,
    double defaultRotVelocityRadPS,
    double headingP,
    double headingI,
    double headingD) {}
