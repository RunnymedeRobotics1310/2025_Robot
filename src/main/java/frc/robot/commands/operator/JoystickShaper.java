package frc.robot.commands.operator;

public final class JoystickShaper {
  public static final double DEFAULT_TRANSLATE_DEADBAND = 0.10; // 0..1
  public static final double DEFAULT_TRANSLATE_EXPO = 0.35; // 0..1 (more = softer near center)

  public static final double DEFAULT_ROTATE_DEADBAND = 0.10; // 0..1
  public static final double DEFAULT_ROTATE_EXPO = 0.35; // 0..1 (more = softer near center)

  public static final double SQUARE_BLEND_START = 0.70; // start blending circle->square here
  public static final double SQUARE_BLEND_END = 1.00; // fully blended by the edge

  private final double translateDeadband;
  private final double translateExpo;

  private final double rotateDeadband;
  private final double rotateExpo;

  /** Create a default shaper */
  public JoystickShaper() {
    this(
        DEFAULT_TRANSLATE_DEADBAND,
        DEFAULT_TRANSLATE_EXPO,
        DEFAULT_ROTATE_DEADBAND,
        DEFAULT_ROTATE_EXPO);
  }

  /** Create a shaper with translation and rotation slew. Set rotRateLimit <= 0 to disable. */
  public JoystickShaper(
      double translateDeadband, double translateExpo, double rotateDeadband, double rotateExpo) {

    this.translateDeadband = translateDeadband;
    this.translateExpo = translateExpo;

    this.rotateDeadband = rotateDeadband;
    this.rotateExpo = rotateExpo;
  }

  /** Shape a 2D stick (e.g., left stick for translation). Returns [x, y] after shaping + slew. */
  public double[] shapeXY(double rawX, double rawY) {
    // 1) radial deadband + rescale to [0..1]
    double r = Math.hypot(rawX, rawY);
    if (r <= translateDeadband) {
      return new double[] {0.0, 0.0};
    }
    double rScaled = (r - translateDeadband) / (1.0 - translateDeadband);
    double nx = (rawX / r) * rScaled;
    double ny = (rawY / r) * rScaled;

    // 2) expo on the magnitude (keep direction)
    double rExpo = applyExpoOnRadius(nx, ny, translateExpo);
    double scaleExpo = (rScaled > 0.0) ? (rExpo / rScaled) : 0.0;
    nx *= scaleExpo;
    ny *= scaleExpo;

    // 3) blend circle->square near the edge so diagonals can reach ±1 cleanly
    return circleToSquareBlend(nx, ny, rExpo);
  }

  /** Shape a single-axis input (e.g., right stick X for rotation), then apply slew if enabled. */
  public double shapeRotation(double rawAxis) {
    double x = applyDeadband1D(rawAxis, rotateDeadband);
    x = applyExpo1D(x, rotateExpo);
    return x;
  }

  // ---------- helpers ----------

  private static double applyDeadband1D(double v, double dz) {
    double a = Math.abs(v);
    if (a <= dz) return 0.0;
    // rescale to [0..1] while preserving sign
    return Math.copySign((a - dz) / (1.0 - dz), v);
  }

  private static double applyExpo1D(double v, double expo) {
    if (expo <= 0.0) return v;
    // Blend linear and cubic: v' = v*(1-e) + v^3*e
    return v * (1.0 - expo) + Math.pow(v, 3) * expo;
  }

  /** Returns new radius after expo, preserving direction. */
  private static double applyExpoOnRadius(double nx, double ny, double expo) {
    double r = Math.hypot(nx, ny);
    if (r == 0.0 || expo <= 0.0) return r;
    return r * (1.0 - expo) + Math.pow(r, 3) * expo;
  }

  /** Blend circle->square only near the edge, so diagonals can hit ±1 without mid-range twitch. */
  private static double[] circleToSquareBlend(double nx, double ny, double r) {
    if (r <= SQUARE_BLEND_START) return new double[] {nx, ny};

    double t =
        (r >= SQUARE_BLEND_END)
            ? 1.0
            : (r - SQUARE_BLEND_START) / (SQUARE_BLEND_END - SQUARE_BLEND_START);

    // Target after square mapping: scale so max(|x|,|y|) == r
    double m = Math.max(Math.abs(nx), Math.abs(ny));
    if (m == 0.0) return new double[] {nx, ny};
    double s = r / m; // scale needed to make the dominant axis reach radius r
    double sx = nx * s;
    double sy = ny * s;

    // Blend between round (nx,ny) and square-mapped (sx,sy)
    double ox = nx * (1.0 - t) + sx * t;
    double oy = ny * (1.0 - t) + sy * t;
    return new double[] {ox, oy};
  }
}
