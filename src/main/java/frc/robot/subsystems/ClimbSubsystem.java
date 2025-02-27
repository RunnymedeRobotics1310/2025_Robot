package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants;

public class ClimbSubsystem extends SubsystemBase {

    private DoubleSolenoid climbPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ClimbConstants.CLIMB_FORWARDS_PNEUMATIC_PORT, ClimbConstants.CLIMB_REVERSE_PNEUMATIC_PORT);


    public ClimbSubsystem() {
        setClimbDeployed(false);
    }

    /**
     *
     *
     *
     * @param deployClimb {@code true} to deploy the climber (climb), {@code false} to retract the
     * climber (lower the robot)
     * 
     * 
     * 
     */

    public void setClimbDeployed(boolean deployClimb) {
        if (deployClimb) {
            climbPiston.set(DoubleSolenoid.Value.kForward);
        } else {
            climbPiston.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean isClimbDeployed() {
        return climbPiston.get() == DoubleSolenoid.Value.kForward;
    }


    /*
     * Periodic routines
     */
    @Override
    public void periodic() {

        checkSafety();


        SmartDashboard.putBoolean("Climb Deployed", isClimbDeployed());
        SmartDashboard.putBoolean("Can deploy climb", true);

    }

    private void checkSafety() {

        // Are there any safety checks?
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("climb: ").append(isClimbDeployed());

        return sb.toString();
    }
}
