// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.telemetry.Telemetry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double lastDashUpdate = 0;

  private double periodicDisabledTime = 0;
  private double periodicTeleopTime = 0;
  private double initDisabledTime = 0;
  private double initTeleopTime = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Add limelights to port forwarding for USB access
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "10.13.10.11", port);
      PortForwarder.add(port + 100, "10.13.10.12", port);
    }

    // This is solely here to trigger Java's dumbness on the first string + double printout delay
    System.out.println("Robot Initialized.  Here's a Random: " + Math.random());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Update telemetry every 150ms
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastDashUpdate > 0.150) {
      Telemetry.post();
      lastDashUpdate = currentTime;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    this.initDisabledTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("1310/Robot/InitDisabledTime", initDisabledTime);
  }

  @Override
  public void disabledPeriodic() {
    this.periodicDisabledTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("1310/Robot/PeriodicDisabledTime", periodicDisabledTime);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    initTeleopTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("1310/Robot/InitTeleopTime", initTeleopTime);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    this.periodicTeleopTime = Timer.getFPGATimestamp();
    double matchTime = DriverStation.getMatchTime();

    // Gimme some stats on this
    SmartDashboard.putNumber("1310/Robot/MatchTime", matchTime);
    SmartDashboard.putNumber("1310/Robot/PeriodicTeleopTime", periodicTeleopTime);
    SmartDashboard.putNumber(
        "1310/Robot/c-SinceLastDisable", periodicDisabledTime - periodicTeleopTime);
    SmartDashboard.putNumber("1310/Robot/c-SinceTeleopInit", initTeleopTime - periodicTeleopTime);
    SmartDashboard.putNumber("1310/Robot/c-MatchTimeCountUp", 135 - matchTime);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
