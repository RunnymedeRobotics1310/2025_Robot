package frc.robot.commands.operator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants.CoralPose;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.climb.AutoClimbCommand;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.coral.MoveToCoralPoseCommand;
import frc.robot.commands.coral.intake.IntakeCoralCommand;
import frc.robot.commands.pneumatics.ToggleCompressorCommand;
import frc.robot.commands.swervedrive.DriveToVisibleTagCommand;
import frc.robot.commands.swervedrive.SetAllianceGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

/** The DriverController exposes all driver functions */
public class OperatorInput extends SubsystemBase {

  private final XboxController driverController;
  private final XboxController operatorController;
  private final SwerveSubsystem swerve;
  private final CoralSubsystem coral;
  private final LimelightVisionSubsystem vision;

  private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.Delay> delayChooser =
      new SendableChooser<>();

  /*  private final SendableChooser<Constants.AutoConstants.ReefLocation1> reefLocation1Chooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.ReefLocation2> reefLocation2Chooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.ReefLocation3> reefLocation3Chooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.ReefPosition1> reefPosition1Chooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.ReefPosition2> reefPosition2Chooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.ReefPosition3> reefPosition3Chooser =
      new SendableChooser<>(); */

  /**
   * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
   *
   * @param driverControllerPort on the driver station which the driver joystick is plugged into
   * @param operatorControllerPort on the driver station which the aux joystick is plugged into
   */
  public OperatorInput(
      int driverControllerPort,
      int operatorControllerPort,
      double deadband,
      SwerveSubsystem swerve,
      CoralSubsystem coral,
      LimelightVisionSubsystem vision) {
    driverController = new GameController(driverControllerPort, deadband);
    operatorController = new GameController(operatorControllerPort, deadband);
    this.swerve = swerve;
    this.coral = coral;
    this.vision = vision;
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
      LimelightVisionSubsystem visionSubsystem) {

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
                this, driveSubsystem, coralSubsystem, pneumaticsSubsystem, climbSubsystem));

    // Reset Gyro
    new Trigger(() -> driverController.getBackButton())
        .onTrue(new SetAllianceGyroCommand(driveSubsystem, 0));

    // Set Yaw
    // TODO: Remove!  Practice Field Only!
    new Trigger(() -> operatorController.getBackButton())
        .onTrue(new SetAllianceGyroCommand(driveSubsystem, -60));

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
     * Set remove algae poses
     */
    new Trigger(
            () -> operatorController.getPOV() == 270 && operatorController.getRightBumperButton())
        .onTrue(new MoveToCoralPoseCommand(CoralPose.REMOVE_HIGH_ALGAE, coralSubsystem));
    new Trigger(
            () -> operatorController.getPOV() == 180 && operatorController.getRightBumperButton())
        .onTrue(new MoveToCoralPoseCommand(CoralPose.REMOVE_LOW_ALGAE, coralSubsystem));

    /*
     * Coral Intake Buttons
     */
    new Trigger(() -> isAlignLeftStation() || isAlignRightStation())
        .onTrue(new IntakeCoralCommand(coralSubsystem, false));

    new Trigger(() -> driverController.getYButton())
        .onTrue(new IntakeCoralCommand(coralSubsystem, true));

    /*
     * Climb Buttons
     */

    // climb
    new Trigger(() -> driverController.getPOV() == 0)
        .onTrue(new ClimbCommand(true, climbSubsystem));

    // anti-climb
    new Trigger(() -> driverController.getPOV() == 180)
        .onTrue(new ClimbCommand(false, climbSubsystem));

    new Trigger(() -> driverController.getPOV() == 270)
        .onTrue(new AutoClimbCommand(climbSubsystem));

    new Trigger(() -> Timer.getMatchTime() < 15 && RobotState.isTeleop())
        .onTrue(new AutoClimbCommand(climbSubsystem));

    new Trigger(this::isToggleCompressor).onTrue(new ToggleCompressorCommand(pneumaticsSubsystem));

    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
        .onTrue(new DriveToVisibleTagCommand(driveSubsystem, visionSubsystem, true));
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

  public boolean isSlowMode() {
    return driverController.getLeftBumperButton();
  }

  public boolean isFaceReef() {
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

  // ALIGN CORAL STATION ANGLE
  public boolean isAlignLeftStation() {
    return driverController.getLeftTriggerAxis() > 0.5;
  }

  public boolean isAlignRightStation() {
    return driverController.getRightTriggerAxis() > 0.5;
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

  public void initAutoSelectors() {

    SmartDashboard.putData("1310/auto/Auto Selector", autoPatternChooser);

    autoPatternChooser.setDefaultOption(
        "Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);
    autoPatternChooser.addOption("Exit Zone", Constants.AutoConstants.AutoPattern.EXIT_ZONE);
    autoPatternChooser.addOption(
        "1 Coral Center", Constants.AutoConstants.AutoPattern.SCORE_1_CENTER);
    autoPatternChooser.addOption("3 Coral Left", Constants.AutoConstants.AutoPattern.SCORE_3_LEFT);
    autoPatternChooser.addOption(
        "3 Coral Right", Constants.AutoConstants.AutoPattern.SCORE_3_RIGHT);

    SmartDashboard.putData("1310/auto/Delay Selector", delayChooser);

    delayChooser.setDefaultOption("No Delay", Constants.AutoConstants.Delay.NO_DELAY);
    delayChooser.addOption("1/2 Seconds", Constants.AutoConstants.Delay.WAIT_0_5_SECOND);
    delayChooser.addOption("1 Second", Constants.AutoConstants.Delay.WAIT_1_SECOND);
    delayChooser.addOption("1 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_1_5_SECONDS);
    delayChooser.addOption("2 Seconds", Constants.AutoConstants.Delay.WAIT_2_SECONDS);
    delayChooser.addOption("2 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_2_5_SECONDS);
    delayChooser.addOption("3 Seconds", Constants.AutoConstants.Delay.WAIT_3_SECONDS);
    delayChooser.addOption("5 Seconds", Constants.AutoConstants.Delay.WAIT_5_SECONDS);
  }

  public Command getAutonomousCommand() {

    double delay =
        switch (delayChooser.getSelected()) {
          case WAIT_0_5_SECOND -> 0.5;
          case WAIT_1_SECOND -> 1;
          case WAIT_1_5_SECONDS -> 1.5;
          case WAIT_2_SECONDS -> 2;
          case WAIT_2_5_SECONDS -> 2.5;
          case WAIT_3_SECONDS -> 3;
          case WAIT_5_SECONDS -> 5;
          default -> 0;
        };

    return switch (autoPatternChooser.getSelected()) {
      case EXIT_ZONE -> new ExitZoneAutoCommand(swerve, delay);
      case SCORE_3_LEFT -> new Score3L4LeftAutoCommand(swerve, coral, vision, delay);
      case SCORE_3_RIGHT -> new Score3L4RightAutoCommand(swerve, coral, vision, delay);
      case SCORE_1_CENTER -> new Score1CoralCenterAutoCommand(swerve, coral, vision, delay);

      default -> new InstantCommand();
    };
  }
}
