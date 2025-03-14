package frc.robot.subsystems.vision;

/**
 * Configuration for the vision system.
 *
 * @param pipelineAprilTagDetect The limelight pipeline id containing april tag detection
 * @param camModeVision the camera mode for vision processing
 * @param fieldExtentMetresX the extent of the field in the x direction in metres
 * @param fieldExtentMetresY the extent of the field in the y direction in metres
 * @param maxAmbiguity the maximum ambiguity allowed for a vision target, above which results will
 *     be ignored
 * @param highQualityAmbiguity the ambiguity value above which the vision data is considered high
 *     quality
 * @param maxVisposeDeltaDistanceMetres the maximum distance between two vispose measurements to be
 *     considered the same
 * @param megatag2 Should megatag2 be used?
 * @param telemetryEnabled whether telemetry is enabled
 */
public record VisionConfig(
    long pipelineAprilTagDetect,
    long camModeVision,
    double fieldExtentMetresX,
    double fieldExtentMetresY,
    double maxAmbiguity,
    double highQualityAmbiguity,
    double maxVisposeDeltaDistanceMetres,
    boolean megatag2,
    boolean telemetryEnabled) {}
