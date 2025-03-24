package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LightingSubsystem extends SubsystemBase {

  private final BooleanTopic climbStatusTopic;

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  private final LEDPattern rainbowLEDPattrn = LEDPattern.rainbow(255, 128);
  private final LEDPattern yellowLEDColour = LEDPattern.solid(Color.kYellow);

  private final AddressableLEDBufferView botPOSLEDView;
  private final AddressableLEDBufferView leftLEDView;
  private final AddressableLEDBufferView rightLEDView;

  public LightingSubsystem() {
    ledStrip = new AddressableLED(Constants.LightingConstants.LED_STRING_PWM_PORT);
    ledStrip.setLength(60);
    ledBuffer = new AddressableLEDBuffer(Constants.LightingConstants.LED_STRING_LENGTH);

    final NetworkTable lightingNetworkTable = NetworkTableInstance.getDefault().getTable("lightingNetworkTable");
    climbStatusTopic = lightingNetworkTable.getBooleanTopic("climbStatusTopic");

    // these values have not been checked
    botPOSLEDView = ledBuffer.createView(20, 39);
    leftLEDView = ledBuffer.createView(0, 19);
    rightLEDView = ledBuffer.createView(40, 59);
  }

  public void periodic() {
    System.out.println();
  }

}
