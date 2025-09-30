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

  /**
   * Construct a TorontoCodingCollectiveGameController on the specified port
   *
   * @param port on the driver station which the joystick is plugged into
   */
  public GameController(int port) {
    super(port);
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

    // The Y axis values should be inverted in order to make the direction away from the driver
    // positive.
    if (axis == XboxController.Axis.kLeftY.value || axis == XboxController.Axis.kRightY.value) {
      axisValue *= -1.0;
    }

    return axisValue;
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
