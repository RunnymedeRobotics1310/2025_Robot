package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;

public class LightingSubsystem extends SubsystemBase {

  public static boolean isClimbing = false;
  public static boolean coralInIntake = false;
  public static boolean isIntaking = false;

  private static final AddressableLED ledStrip =
      new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);
  private static final AddressableLEDBuffer ledBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);
  private static final AddressableLEDBuffer ledClimbingBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);
  private static final AddressableLEDBuffer ledIntakeBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);
  private static final AddressableLEDBuffer ledEndgameBuffer =
      new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

  private static final LEDPattern rainbowLEDPattrn = LEDPattern.rainbow(255, 128);
  // private static final Distance kLedSpacing = Meters.of(1 / 120.0);
  // private final LEDPattern m_scrollingRainbow =
  // m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
  private static final LEDPattern yellowLEDPatern = LEDPattern.solid(Color.kYellow);
  private static final LEDPattern greenLedPattern = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern whiteLedPattern = LEDPattern.solid(Color.kWhite);
  private static final LEDPattern purpleLedPattern = LEDPattern.solid(Color.kPurple);
  private static final LEDPattern redLedPattern = LEDPattern.solid(Color.kRed);
  private static final LEDPattern blueLedPattern = LEDPattern.solid(Color.kBlue);

  public LightingSubsystem() {
    ledStrip.setLength(Constants.LightingConstants.LED_STRING_LENGTH);
    ledStrip.start();
  }

  @Override
  public void periodic() {

    if (isClimbing) {
      rainbowLEDPattrn.applyTo(ledClimbingBuffer);
      ledStrip.setData(ledClimbingBuffer);
    } else if (DriverStation.getMatchTime() > 120 && DriverStation.getMatchTime() < 122) {
      purpleLedPattern.applyTo(ledEndgameBuffer);
      ledStrip.setData(ledEndgameBuffer);
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
  }
}
