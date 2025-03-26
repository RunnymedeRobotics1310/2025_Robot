package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

  private static final LEDPattern rainbowLEDPattrn = LEDPattern.rainbow(255, 128);
  private static final LEDPattern yellowLEDPatern = LEDPattern.solid(Color.kYellow);
  private static final LEDPattern greenLedPattern = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern whiteLedPattern = LEDPattern.solid(Color.kWhite);

  public LightingSubsystem() {
    ledStrip.setLength(Constants.LightingConstants.LED_STRING_LENGTH);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public void periodic() {

    greenLedPattern.applyTo(ledClimbingBuffer);

    if (isClimbing) {
      rainbowLEDPattrn.applyTo(ledClimbingBuffer);
    } else if (DriverStation.getMatchTime() > 120 && DriverStation.getMatchTime() < 122) {

    } else if (coralInIntake) {
      whiteLedPattern.applyTo(ledIntakeBuffer);
    } else if (isIntaking) {
      yellowLEDPatern.applyTo(ledIntakeBuffer);
    }
  }

  //   public static void startClimbLed() {
  //     rainbowLEDPattrn.applyTo(ledClimbingBuffer);
  //   }

  //   public static void setCoralInIntakeLed(boolean coralInBot) {
  //     if (coralInBot) {
  //       greenLedPattern.applyTo(ledIntakeBuffer);
  //     }
  //   }

  //   public static void setIntakingLed(boolean isIntaking) {
  //     if(isIntaking){
  //         coralLedPattern.applyTo(ledIntakeBuffer);
  //     }
  // }
}
