package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  private final LEDPattern rainbowLEDPattrn = LEDPattern.rainbow(255, 128);
  private final LEDPattern yellowLEDColour = LEDPattern.solid(Color.kYellow);

  private final AddressableLEDBufferView botPOSLEDView;
  private final AddressableLEDBufferView leftLEDView;
  private final AddressableLEDBufferView rightLEDView;

  public void periodic() {
    if (DriverStation.getMatchTime() > 100 && !DriverStation.isAutonomous()) {
      yellowLEDColour.applyTo(leftLEDView);
      yellowLEDColour.applyTo(rightLEDView);
      System.out.println("Weeeeee");
    }
  }

  public void setClimbLEDPattern(boolean climbing) {

    if (climbing) {
      rainbowLEDPattrn.applyTo(leftLEDView);
      rainbowLEDPattrn.applyTo(rightLEDView);
    } else {
      LEDPattern.kOff.applyTo(leftLEDView);
      LEDPattern.kOff.applyTo(rightLEDView);
    }
  }

  public LightingSubsystem() {
    ledStrip = new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);
    ledStrip.setLength(60);
    ledBuffer = new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

    // these values have not been checked
    botPOSLEDView = ledBuffer.createView(20, 39);
    leftLEDView = ledBuffer.createView(0, 19);
    rightLEDView = ledBuffer.createView(40, 59);
  }
}
