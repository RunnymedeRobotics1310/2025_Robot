package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The TorontoCodingCollective Game Controller extends {@link XboxController}
 *
 * <p>This class adds deadbanding to the axes values (X,Y) of the left and right joysticks on the
 * XBox controller, as well as the Triggers
 *
 * <p>Deadbanding of the axis values is intended to prevent 'drift' or movement of the robot when
 * the operators are not touching the controls.
 *
 * <p>Since the TccGameController overrides the {@link XboxController#getRawAxis} method, an
 * additional method {@link #getHardwareAxisValue} is provided to retrieve the underlying hardware
 * values
 */
public class GameController extends XboxController {

  public static final double DEFAULT_GAME_CONTROLLER_AXIS_DEADBAND = .2;

  private double axisDeadband = DEFAULT_GAME_CONTROLLER_AXIS_DEADBAND;

  /**
   * Construct a TorontoCodingCollectiveGameController on the specified port
   *
   * <p>Uses the {{@link #DEFAULT_GAME_CONTROLLER_AXIS_DEADBAND} as the joystick deadband
   *
   * @param port on the driver station which the joystick is plugged into
   */
  public GameController(int port) {
    this(port, DEFAULT_GAME_CONTROLLER_AXIS_DEADBAND);
  }

  /**
   * Construct a TorontoCodingCollective TccGameController on the specified port with the specified
   * deadband
   *
   * @param port on the driver station which the joystick is plugged into
   * @param axisDeadband (0 - 0.4) to use for all axis values on this controller. When the axis
   *     value from the hardware is less than the specified value, then the axis will return zero.
   *     Setting the axisDeadbanding to zero turns off all deadbanding. Values < 0 or > 0.4 are
   *     ignored, and the {@link #DEFAULT_GAME_CONTROLLER_AXIS_DEADBAND} value is used.
   */
  public GameController(int port, final double axisDeadband) {
    super(port);

    if (axisDeadband < 0 || axisDeadband > 0.4) {
      System.out.println(
          "Invalid axis deadband("
              + axisDeadband
              + ") must be between 0 - 0.4. Overriding value to "
              + DEFAULT_GAME_CONTROLLER_AXIS_DEADBAND);
      setAxisDeadband(DEFAULT_GAME_CONTROLLER_AXIS_DEADBAND);
    } else {
      setAxisDeadband(axisDeadband);
    }
  }

  /**
   * Get the value of the axis with the deadbanding applied.
   *
   * <p>This routine overrides the HID interface to apply deadbanding to the axis values on the
   * underlying XBoxGameController.
   *
   * <p>For the Y-axis of the left and right stick, the value is inverted so that pushing the stick
   * forward (away from the operator) returns a positive Y value instead of negative Y value from
   * the hardware.
   *
   * @param axis The axis to read, starting at 0.
   * @return The value of the axis.
   */
  @Override
  public double getRawAxis(int axis) {

    double axisValue = super.getRawAxis(axis);

    if (Math.abs(axisValue) < axisDeadband) {
      axisValue = 0;
    } else {
      // Subtract the deadband (take the absolute value in order to remove
      // the deadband amount whether it is positive or negative.
      double value = Math.abs(axisValue) - axisDeadband;

      // Scale the value to the full range of 0-1.0 after the deadband amount
      // is removed
      value = value / (1.0 - axisDeadband);

      // multiply by 1.0 or -1.0 in order to put the sign back
      // on the end result based on the original axis value.
      value *= Math.signum(axisValue);

      axisValue = value;
    }

    // The Y axis values should be inverted in order to make the direction away from the driver
    // positive.
    if (axis == XboxController.Axis.kLeftY.value || axis == XboxController.Axis.kRightY.value) {
      axisValue *= -1.0;
    }

    return axisValue;
  }

  /**
   * Set the current axis deadband on the stick and trigger axes of this gameController
   *
   * <p>Use the method {@link #getRawAxis(int)} to get the hardware value coming off the game
   * controller axis before deadbanding.
   *
   * @returns axisDeadband used for all axis values on this controller.
   */
  public double getAxisDeadband() {
    return this.axisDeadband;
  }

  /**
   * Set the axis deadband on the stick and trigger axes of this gameController
   *
   * <p>The value set applies to the x and y axes of the left and right sticks, as well as the
   * trigger's axes values.
   *
   * <p>The deadband must be set larger than the highest expected value returned from the stick axis
   * when they are released. A released controller axis will not always return to zero.
   *
   * <p>Use the method {@link #getRawAxis(int)} to get the hardware value coming off the game
   * controller axis before deadbanding.
   *
   * @param axisDeadband (0 - 0.4) to use for all axis values on this controller. When the axis
   *     value from the hardware is less than the specified value, then the axis will return zero.
   *     Setting the axisDeadbanding to zero turns off all deadbanding.
   */
  public void setAxisDeadband(double axisDeadband) {

    if (axisDeadband < 0 || axisDeadband > 0.4) {
      System.out.println(
          "Invalid axis deadband("
              + axisDeadband
              + ") must be between 0 - 0.4. Axis deadband value not changed.  Currently "
              + this.axisDeadband);
      return;
    }

    this.axisDeadband = axisDeadband;
  }

  /**
   * Get the raw hardware axis value (unmodified by the deadband)
   *
   * @param axis see {@link XboxController.Axis} for list of axis constants
   */
  public double getHardwareAxisValue(int axis) {
    return super.getRawAxis(axis);
  }

  @Override
  public String toString() {

    StringBuilder sb = new StringBuilder();

    /*
     * Axis
     */
    // Left stick
    sb.append('(')
        .append(Math.round(getLeftX() * 100d) / 100d)
        .append(',')
        .append(Math.round(getLeftY() * 100d) / 100d)
        .append(')');

    // Right stick
    sb.append('(')
        .append(Math.round(getRightX() * 100d) / 100d)
        .append(',')
        .append(Math.round(getRightY() * 100d) / 100d)
        .append(')');

    // Triggers
    sb.append('[')
        .append(Math.round(getLeftTriggerAxis() * 100d) / 100d)
        .append(',')
        .append(Math.round(getRightTriggerAxis() * 100d) / 100d)
        .append("] ");

    /*
     * POV
     */
    if (getPOV() >= 0) {
      sb.append("POV(").append(getPOV()).append(") ");
    }

    /*
     * Buttons
     */
    if (getLeftBumperButton()) {
      sb.append("LB ");
    }
    if (getRightBumperButton()) {
      sb.append("RB ");
    }
    if (getAButton()) {
      sb.append("A ");
    }
    if (getBButton()) {
      sb.append("B ");
    }
    if (getXButton()) {
      sb.append("X ");
    }
    if (getYButton()) {
      sb.append("Y ");
    }
    if (getBackButton()) {
      sb.append("Back ");
    }
    if (getBackButton()) {
      sb.append("Start ");
    }
    if (getLeftStickButtonPressed()) {
      sb.append("LStick ");
    }
    if (getRightStickButtonPressed()) {
      sb.append("RStick ");
    }

    return sb.toString();
  }
}
