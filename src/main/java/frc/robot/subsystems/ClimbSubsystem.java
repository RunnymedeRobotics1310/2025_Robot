package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class ClimbSubsystem extends SubsystemBase {

    private Solenoid              firstClimbPiston = new Solenoid(PneumaticsModuleType.CTREPCM,
            Constants.ClimbConstants.CLIMB_FIRST_PNEUMATIC_PORT);
    private Solenoid              secondClimbPiston = new Solenoid(PneumaticsModuleType.CTREPCM,
            Constants.ClimbConstants.CLIMB_SECOND_PNEUMATIC_PORT);


    public ClimbSubsystem() {

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
        firstClimbPiston.set(deployClimb);
        secondClimbPiston.set(deployClimb);
    }

    public boolean isClimbDeployed() {
        return firstClimbPiston.get();
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
