package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants.CoralPose;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.SetupScoreCommand;
import frc.robot.commands.coral.SetupScoreCommandBotPose;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.coral.intake.PlantCoralCommand;
import frc.robot.commands.pneumatics.ToggleCompressorCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/** The DriverController exposes all driver functions */
public class OperatorInput extends SubsystemBase {

  private final XboxController driverController;
  private final XboxController operatorController;

  /**
   * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
   *
   * @param driverControllerPort on the driver station which the driver joystick is plugged into
   * @param operatorControllerPort on the driver station which the aux joystick is plugged into
   */
  public OperatorInput(int driverControllerPort, int operatorControllerPort, double deadband) {
    driverController = new GameController(driverControllerPort, deadband);
    operatorController = new GameController(operatorControllerPort, deadband);
  }

  /**
   * Configure the button bindings for all operator commands
   *
   * <p>NOTE: This routine requires all subsystems to be passed in
   *
   * <p>NOTE: This routine must only be called once from the RobotContainer
   */
  public void configureButtonBindings(
          SwerveSubsystem driveSubsystem,
          CoralSubsystem coralSubsystem,
          PneumaticsSubsystem pneumaticsSubsystem,
          ClimbSubsystem climbSubsystem,
          AlgaeSubsystem algaeSubsystem, LimelightVisionSubsystem visionSubsystem) {

    // System Test Command
    new Trigger(
            () ->
                driverController.getStartButton()
                    && driverController.getBackButton()
                    && !DriverStation.isFMSAttached())
        .onTrue(new SystemTestCommand(this, driveSubsystem, coralSubsystem));

    // Cancel Command
    new Trigger(this::isCancel)
        .whileTrue(
            new CancelCommand(
                this,
                driveSubsystem,
                coralSubsystem,
                pneumaticsSubsystem,
                climbSubsystem,
                algaeSubsystem));

    // Reset Gyro
    new Trigger(() -> driverController.getBackButton()).onTrue(new ZeroGyroCommand(driveSubsystem));

    // Compact (X button)
    new Trigger(() -> driverController.getXButton() || operatorController.getXButton())
        .onTrue(new MoveToCoralPoseCommand(CoralPose.COMPACT, coralSubsystem));

    /*
     * Set Score Height (POV)
     */
    // Y (delivery), A (intake) for arm position
    new Trigger(() -> operatorController.getPOV() == 0)
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L4, coralSubsystem));
    new Trigger(() -> operatorController.getPOV() == 270)
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L3, coralSubsystem));
    new Trigger(() -> operatorController.getPOV() == 180)
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L2, coralSubsystem));
    new Trigger(() -> operatorController.getPOV() == 90)
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L1, coralSubsystem));

    /*
     * Coral Intake Buttons
     */
    new Trigger(() -> driverController.getLeftBumperButton())
        .onTrue(new IntakeCoralCommand(coralSubsystem));

    /*
     * Climb Buttons
     */

    // climb
    new Trigger(() -> driverController.getPOV() == 0)
        .onTrue(new ClimbCommand(true, climbSubsystem));

    // anti-climb
    new Trigger(() -> driverController.getPOV() == 180)
        .onTrue(new ClimbCommand(false, climbSubsystem));

    new Trigger(this::isToggleCompressor).onTrue(new ToggleCompressorCommand(pneumaticsSubsystem));

    //    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
    //        .onTrue(
    //            new SetupScoreCommand(
    //                    CoralPose.SCORE_L4,
    //                    Constants.CoralConstants.DesiredDistanceToTargetCM.LEVEL_4,
    //                    true,
    //                    coralSubsystem,
    //                    driveSubsystem,
    //                    visionSubsystem));

    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
        .onTrue(
            new SetupScoreCommandBotPose(
                    coralSubsystem,
                driveSubsystem,
                visionSubsystem, Constants.AutoConstants.FieldLocation.preScoreBlueRight3,
                CoralPose.SCORE_L4));
  }

  /*
   * Cancel Command support
   * Do not end the command while the button is pressed
   */
  public boolean isCancel() {
    return (driverController.getStartButton() && !driverController.getBackButton())
        || (operatorController.getStartButton());
  }

  public boolean isZeroGyro() {
    return driverController.getBackButton();
  }

  /*
   * Default Drive Command Buttons
   */
  public XboxController getRawDriverController() {
    return driverController;
  }

  public boolean getRotate180Val() {
    return driverController.getAButton();
  }

  /*
   * The following routines are used by the default commands for each subsystem
   *
   * They allow the default commands to get user input to manually move the
   * robot elements.
   */

  public boolean isFastMode() {
    return driverController.getRightBumperButton();
  }

  public boolean isFaceTarget() {
    return driverController.getBButton();
  }

  public double getDriverControllerAxis(Stick stick, Axis axis) {
    switch (stick) {
      case LEFT:
        switch (axis) {
          case X:
            return driverController.getLeftX();
          case Y:
            return driverController.getLeftY();
        }
        break;
      case RIGHT:
        switch (axis) {
          case X:
            return driverController.getRightX();
        }
        break;
    }

    return 0;
  }

  public double getOperatorControllerAxis(Stick stick, Axis axis) {
    switch (stick) {
      case LEFT:
        switch (axis) {
          case X:
            return operatorController.getLeftX();
          case Y:
            return operatorController.getLeftY();
        }
        break;
      case RIGHT:
        switch (axis) {
          case X:
            return operatorController.getRightX();
        }
        break;
    }

    return 0;
  }

  /*
   * Default Coral Command
   */
  public double getElevatorInput() {
    return operatorController.getRightY();
  }

  public double getArmStick() {
    return operatorController.getRightX();
  }

  public boolean getEjectButton() {
    return operatorController.getLeftBumperButton();
  }

  public boolean getInjectButton() {
    return operatorController.getBButton();
  }

  public boolean getPlant() {
    return operatorController.getRightTriggerAxis() > 0.5;
  }

  /*
   * Default Algae Command
   */
  public boolean getIntakeAlgae() {
    return driverController.getLeftTriggerAxis() <= 0.5;
  }

  public boolean getOuttakeAlgae() {
    return driverController.getRightTriggerAxis() <= 0.5;
  }

  /*
   * Compressor enable/disable
   */
  public boolean isToggleCompressor() {
    return operatorController.getRightBumperButton() && operatorController.getAButton();
  }

  /*
   * Support for haptic feedback to the driver
   */
  public void startVibrate() {
    driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
  }

  public void stopVibrate() {
    driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Driver Controller", driverController.toString());
  }

  public enum Stick {
    LEFT,
    RIGHT
  }

  public enum Axis {
    X,
    Y
  }
}
