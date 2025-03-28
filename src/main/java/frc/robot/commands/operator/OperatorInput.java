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
import frc.robot.commands.swervedrive.*;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;
import java.util.function.Consumer;

/** The DriverController exposes all driver functions */
public class OperatorInput extends SubsystemBase {

  private final XboxController driverController;
  private final XboxController operatorController;
  private final SwerveSubsystem swerve;
  private final CoralSubsystem coral;
  private final LimelightVisionSubsystem vision;

  private Command autonomousCommand = new InstantCommand();

  private boolean matchNearEndTimerStarted = false;

  public enum RumblePattern {
    NONE(0, XboxController.RumbleType.kBothRumble, true, true),
    BLIP(0.25, XboxController.RumbleType.kBothRumble, true, true),
    SHORT(0.5, XboxController.RumbleType.kBothRumble, true, true),
    MEDIUM(1, XboxController.RumbleType.kBothRumble, true, true),
    RED_ALERT(2, XboxController.RumbleType.kBothRumble, true, true),
    TAG_ALIGN_LEFT(0.5, XboxController.RumbleType.kLeftRumble, false, true),
    TAG_ALIGN_RIGHT(0.5, XboxController.RumbleType.kRightRumble, false, true);

    public final double seconds;
    public final XboxController.RumbleType rumbleType;
    public final boolean driverController;
    public final boolean operatorController;

    RumblePattern(
        double seconds,
        XboxController.RumbleType rumbleType,
        boolean driverController,
        boolean operatorController) {
      this.seconds = seconds;
      this.rumbleType = rumbleType;
      this.driverController = driverController;
      this.operatorController = operatorController;
    }
  }

  private static RumblePattern currentRumblePattern = RumblePattern.NONE;
  private static final Timer rumbleTimer = new Timer();

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
    new Trigger(() -> operatorController.getBackButton())
        .onTrue(new SetAllianceGyroCommand(driveSubsystem, 180));

    // Compact (X button)
    new Trigger(() -> driverController.getXButton() || operatorController.getXButton())
        .onTrue(new MoveToCoralPoseCommand(CoralPose.COMPACT, coralSubsystem));

    /*
     * Set Score Height (POV)
     */
    // Y (delivery), A (intake) for arm position
    new Trigger(() -> operatorController.getPOV() == 0)
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L4, coralSubsystem));
    new Trigger(() -> operatorController.getPOV() == 270 && !isAutoAlignReef())
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L3, coralSubsystem));
    new Trigger(() -> operatorController.getPOV() == 180)
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L2, coralSubsystem));
    new Trigger(() -> operatorController.getPOV() == 90 && !isAutoAlignReef())
        .onTrue(new MoveToCoralPoseCommand(CoralPose.SCORE_L1, coralSubsystem));

    /*
     * Set remove algae poses
     */
    new Trigger(
            () ->
                operatorController.getPOV() == 270
                    && operatorController.getRightBumperButton()
                    && !isAutoAlignReef())
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

    new Trigger(() -> (isAutoAlignReef() && operatorController.getPOV() == 270))
        .onTrue(new DriveToVisibleTagCommand(driveSubsystem, visionSubsystem, true));
    //                .alongWith(new MoveToCoralPoseCommand(CoralPose.SCORE_L4, coral)));

    new Trigger(() -> (isAutoAlignReef() && operatorController.getPOV() == 90))
        .onTrue(new DriveToVisibleTagCommand(driveSubsystem, visionSubsystem, false));
    //                            .alongWith(new MoveToCoralPoseCommand(CoralPose.SCORE_L4,
    // coral)));
  }

  public void configureDashboardBindings(
      SwerveSubsystem driveSubsystem,
      CoralSubsystem coralSubsystem,
      PneumaticsSubsystem pneumaticsSubsystem,
      ClimbSubsystem climbSubsystem,
      LimelightVisionSubsystem visionSubsystem) {

    // Drive To Reef Buttons
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-L1",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_1));
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-L2",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_2));
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-L3",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_3));
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-L4",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_LEFT_4));
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-R1",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_1));
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-R2",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_2));
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-R3",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_3));
    SmartDashboard.putData(
        "1310/Commands/ReefTagCommand-R4",
        new DriveToReefTagCommand(
            swerve, vision, Constants.AutoConstants.FieldLocation.PRE_SCORE_RIGHT_4));

    // Auto Start poses
    SmartDashboard.putData(
        "1310/Commands/AutoStart-Left",
        new MoveToCoralPoseCommand(CoralPose.COMPACT, coral)
            .alongWith(
                new DriveThroughFieldLocationCommand(
                        swerve,
                        Constants.AutoConstants.FieldLocation.AUTO_START_LEFT,
                        1,
                        false,
                        0.02,
                        0.6)
                    .andThen(new NullDriveCommand(swerve).withTimeout(0.1))));
    SmartDashboard.putData(
        "1310/Commands/AutoStart-Right",
        new MoveToCoralPoseCommand(CoralPose.COMPACT, coral)
            .alongWith(
                new DriveThroughFieldLocationCommand(
                        swerve,
                        Constants.AutoConstants.FieldLocation.AUTO_START_RIGHT,
                        1,
                        false,
                        0.02,
                        0.6)
                    .andThen(new NullDriveCommand(swerve).withTimeout(0.1))));

    // Coral Commands
    SmartDashboard.putData(
        "1310/Commands/Coral-Intake", new IntakeCoralCommand(coralSubsystem, false));
    SmartDashboard.putData(
        "1310/Commands/Coral-Compact", new MoveToCoralPoseCommand(CoralPose.COMPACT, coral));
    SmartDashboard.putData(
        "1310/Commands/Coral-L2", new MoveToCoralPoseCommand(CoralPose.SCORE_L2, coral));
    SmartDashboard.putData(
        "1310/Commands/Coral-L3", new MoveToCoralPoseCommand(CoralPose.SCORE_L3, coral));
    SmartDashboard.putData(
        "1310/Commands/Coral-L4", new MoveToCoralPoseCommand(CoralPose.SCORE_L4, coral));
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

  public boolean is180Gyro() {
    return operatorController.getBackButton();
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

  public boolean isAutoAlignReef() {
    return operatorController.getLeftTriggerAxis() > 0.1;
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
    operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
  }

  public void stopVibrate() {
    driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    if (Constants.TelemetryConfig.oi) {
      SmartDashboard.putString("Driver Controller", driverController.toString());
    }

    if (RobotState.isTeleop()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= 20
        && !matchNearEndTimerStarted) {
      setRumblePattern(RumblePattern.SHORT);
      matchNearEndTimerStarted = true;
    }

    rumbleUpdate();
  }

  public enum Stick {
    LEFT,
    RIGHT
  }

  public enum Axis {
    X,
    Y
  }

  public static void setRumblePattern(RumblePattern pattern) {
    if (pattern != currentRumblePattern) {
      currentRumblePattern = pattern;
      rumbleTimer.restart();
    }
  }

  private void rumbleUpdate() {
    if (!rumbleTimer.isRunning()) {
      return;
    }

    double time = rumbleTimer.get();
    double rumbleAmount = 1.0;

    // stop after rumble duration seconds
    if (time > currentRumblePattern.seconds
        || DriverStation.isDisabled()
        || !DriverStation.isTeleop()) {
      currentRumblePattern = RumblePattern.NONE;
      rumbleTimer.stop();
      rumbleAmount = 0.0;
    }

    if (currentRumblePattern.driverController) {
      driverController.setRumble(currentRumblePattern.rumbleType, rumbleAmount);
    }
    if (currentRumblePattern.operatorController) {
      operatorController.setRumble(currentRumblePattern.rumbleType, rumbleAmount);
    }
  }

  public void initAutoSelectors() {

    SmartDashboard.putData("1310/auto/Auto Selector", autoPatternChooser);

    autoPatternChooser.setDefaultOption(
        "3 Coral Left", Constants.AutoConstants.AutoPattern.SCORE_3_LEFT);
    autoPatternChooser.addOption("Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);
    autoPatternChooser.addOption("Exit Zone", Constants.AutoConstants.AutoPattern.EXIT_ZONE);
    autoPatternChooser.addOption(
        "1 Coral Center", Constants.AutoConstants.AutoPattern.SCORE_1_CENTER);
    autoPatternChooser.addOption(
        "3 Coral Right", Constants.AutoConstants.AutoPattern.SCORE_3_RIGHT);

    autoPatternChooser.onChange(
        new Consumer<Constants.AutoConstants.AutoPattern>() {
          @Override
          public void accept(Constants.AutoConstants.AutoPattern pattern) {
            autonomousCommand = generateAutonomousCommand(delayChooser.getSelected(), pattern);
          }
        });

    SmartDashboard.putData("1310/auto/Delay Selector", delayChooser);

    delayChooser.setDefaultOption("No Delay", Constants.AutoConstants.Delay.NO_DELAY);
    delayChooser.addOption("1/2 Seconds", Constants.AutoConstants.Delay.WAIT_0_5_SECOND);
    delayChooser.addOption("1 Second", Constants.AutoConstants.Delay.WAIT_1_SECOND);
    delayChooser.addOption("1 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_1_5_SECONDS);
    delayChooser.addOption("2 Seconds", Constants.AutoConstants.Delay.WAIT_2_SECONDS);
    delayChooser.addOption("2 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_2_5_SECONDS);
    delayChooser.addOption("3 Seconds", Constants.AutoConstants.Delay.WAIT_3_SECONDS);
    delayChooser.addOption("5 Seconds", Constants.AutoConstants.Delay.WAIT_5_SECONDS);

    delayChooser.onChange(
        new Consumer<Constants.AutoConstants.Delay>() {
          @Override
          public void accept(Constants.AutoConstants.Delay delay) {
            autonomousCommand = generateAutonomousCommand(delay, autoPatternChooser.getSelected());
          }
        });

    // Set default autonomous command
    autonomousCommand =
        generateAutonomousCommand(delayChooser.getSelected(), autoPatternChooser.getSelected());
  }

  private Command generateAutonomousCommand(
      Constants.AutoConstants.Delay delayChoice,
      Constants.AutoConstants.AutoPattern patternChoice) {

    double delay =
        switch (delayChoice) {
          case WAIT_0_5_SECOND -> 0.5;
          case WAIT_1_SECOND -> 1;
          case WAIT_1_5_SECONDS -> 1.5;
          case WAIT_2_SECONDS -> 2;
          case WAIT_2_5_SECONDS -> 2.5;
          case WAIT_3_SECONDS -> 3;
          case WAIT_5_SECONDS -> 5;
          default -> 0;
        };

    return switch (patternChoice) {
      case EXIT_ZONE -> new ExitZoneAutoCommand(swerve, delay);
      case SCORE_3_LEFT -> new Score3L4LeftAutoCommand(swerve, coral, vision, delay);
      case SCORE_3_RIGHT -> new Score3L4RightAutoCommand(swerve, coral, vision, delay);
      case SCORE_1_CENTER -> new Score1CoralCenterAutoCommand(swerve, coral, vision, delay);

      default -> new InstantCommand();
    };
  }

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }
}
