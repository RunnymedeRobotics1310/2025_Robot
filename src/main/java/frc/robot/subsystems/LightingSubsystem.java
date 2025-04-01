package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.telemetry.Telemetry;

public class LightingSubsystem extends SubsystemBase {

  public static boolean isClimbDeployed = false;
  public static boolean isCageInPosition = false;
  public static boolean coralInIntake = false;
  public static boolean isIntaking = false;
  public static int flashCount = 0;

  public static Timer ledTimerOn = new Timer();
  public static Timer ledTimerOff = new Timer();

  private static final AddressableLED ledStrip =
      new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);
  private static final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

  // Match buffers
  private static final AddressableLEDBuffer ledClimbingBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);
  private static final AddressableLEDBuffer ledIntakeBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);
  private static final AddressableLEDBuffer ledEndgameBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

  // Debuging buffers
  private static final AddressableLEDBuffer ledRobotHealthBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

  // LED patterns
  private static final LEDPattern rainbowLedPattern = LEDPattern.rainbow(255, 128);
  private static final Distance kLedSpacing = Units.Meters.of(1 / 120.0);
  private final LEDPattern scrollingRainbowLedPattern =
      rainbowLedPattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(0.5), kLedSpacing);
  private static final LEDPattern yellowLEDPatern = LEDPattern.solid(Color.kYellow);
  private static final LEDPattern greenLedPattern = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern whiteLedPattern = LEDPattern.solid(Color.kWhite);
  private static final LEDPattern purpleLedPattern = LEDPattern.solid(Color.kPurple);
  private static final LEDPattern redLedPattern = LEDPattern.solid(Color.kRed);
  private static final LEDPattern blueLedPattern = LEDPattern.solid(Color.kBlue);
  private static final LEDPattern orangeLedPattern = LEDPattern.solid(Color.kOrange);

  public LightingSubsystem() {
    ledStrip.setLength(Constants.LightingConstants.LED_STRING_LENGTH);
    ledStrip.start();

    ledTimerOn.reset();
    ledTimerOn.start();
    ledTimerOff.reset();
  }

  @Override
  public void periodic() {

    if (DriverStation.isEnabled()) {
      if (isClimbDeployed) {
        scrollingRainbowLedPattern.applyTo(ledClimbingBuffer);
        ledStrip.setData(ledClimbingBuffer);
      } else if (isCageInPosition) {
        rainbowLedPattern.applyTo(ledClimbingBuffer);
        ledStrip.setData(ledClimbingBuffer);
      } else if (DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 17) {
        flashLed(ledEndgameBuffer, purpleLedPattern);
      } else if (coralInIntake) {
        whiteLedPattern.applyTo(ledIntakeBuffer);
        ledStrip.setData(ledIntakeBuffer);
      } else if (isIntaking) {
        yellowLEDPatern.applyTo(ledIntakeBuffer);
        ledStrip.setData(ledIntakeBuffer);
      } else {
        if (RunnymedeUtils.getRunnymedeAlliance() == Alliance.Red) {
          redLedPattern.applyTo(ledBuffer);
          ledStrip.setData(ledBuffer);
        } else {
          blueLedPattern.applyTo(ledBuffer);
          ledStrip.setData(ledBuffer);
        }
      }
    } else {
      if (!Telemetry.healthyRobot) {
        flashLed(ledRobotHealthBuffer, redLedPattern);
      } else {
        LEDPattern.kOff.applyTo(ledRobotHealthBuffer);
        ledStrip.setData(ledRobotHealthBuffer);
      }
    }
  }

  private void flashLed(AddressableLEDBuffer buffer, LEDPattern pattern) {
    if (ledTimerOn.get() >= .1) {
      ledTimerOn.reset();
      ledTimerOn.stop();
      ledTimerOff.start();

      pattern.applyTo(buffer);
      ledStrip.setData(buffer);
    } else if (ledTimerOff.get() >= .1) {
      ledTimerOff.reset();
      ledTimerOff.stop();
      ledTimerOn.start();

      LEDPattern.kOff.applyTo(buffer);
      ledStrip.setData(buffer);
    }
  }
}
