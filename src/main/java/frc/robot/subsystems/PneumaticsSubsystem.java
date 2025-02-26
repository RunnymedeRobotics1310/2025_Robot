package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {

    private final Compressor compresor = new Compressor(PneumaticsModuleType.CTREPCM);

    public PneumaticsSubsystem() {

    }


    public void setCompresor(boolean enabled) {
        if (enabled) {
            compresor.enableDigital();
        } else if (!enabled) {
            compresor.disable();
        }
    }

    public boolean getCompressorEnabled() {
        return compresor.isEnabled();
    }

    public void periodic() {
        SmartDashboard.putBoolean("Compressor Enabled", compresor.isEnabled());
        SmartDashboard.putBoolean("Compressor On", compresor.getPressureSwitchValue());
    }

}
