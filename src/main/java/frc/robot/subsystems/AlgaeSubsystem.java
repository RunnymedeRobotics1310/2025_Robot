package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AlgaeConstants.ALGAE_FORWARDS_PNEUMATIC_PORT;
import static frc.robot.Constants.AlgaeConstants.ALGAE_REVERSE_PNEUMATIC_PORT;


public class AlgaeSubsystem extends SubsystemBase {

    // Algae Subsystem Motors
//    private final SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR_CAN_ID, SparkLowLevel.MotorType.kBrushless);

    private double intakeSpeed = 0;

    private DoubleSolenoid algaeArmPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            ALGAE_FORWARDS_PNEUMATIC_PORT, ALGAE_REVERSE_PNEUMATIC_PORT);


    public AlgaeSubsystem() {

    }

    /*
     * Intake Routines
     */


    public void setIntakeSpeed(double speed) {

        this.intakeSpeed = speed;

        checkSafety();

//        intakeMotor.set(intakeSpeed);

    }

    /*
     * Arm Routines
     */

    public void deployArm(boolean shift) {
        if (shift) {
            algaeArmPiston.set(DoubleSolenoid.Value.kForward);
        } else {
            algaeArmPiston.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean isArmDeployed() {
        return algaeArmPiston.get() == DoubleSolenoid.Value.kForward;
    }

    public void stop() {
        setIntakeSpeed(0);
    }

    /*
     * Periodic routines
     */
    @Override
    public void periodic() {

        checkSafety();

        SmartDashboard.putNumber("Algae Intake Motor", intakeSpeed);
        SmartDashboard.putBoolean("Algae Arm Deployed", isArmDeployed());

    }

    private void checkSafety() {

        // Are there any safety checks?
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
                .append("intake: speed ").append(intakeSpeed).append("arm deployed: ").append(isArmDeployed());

        return sb.toString();
    }
}
