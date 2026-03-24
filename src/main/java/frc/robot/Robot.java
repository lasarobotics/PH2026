// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private final LoopLogger ll = new LoopLogger();

  private boolean hasRunTeleop = false;

  private RobotContainer m_robotContainer;

  public AutoHoncho autoHoncho;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotInit() {
    if (isSimulation()) {
      NetworkTableInstance.getDefault().startServer();
    }

    // advnatage kit logging
    LoggingInitializer.getInstance();
    SignalLogger.enableAutoLogging(false);

    RobotController.setBrownoutVoltage(6.25);

    m_robotContainer = new RobotContainer();

    // initialize subsystems
    DriveSubsystem.getInstance();
    IntakeSubsystem.getInstance();
    ShooterSubsystem.getInstance();
    LEDSubsystem.getInstance();
    
    IntakeSubsystem.getInstance().deployIntake();
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
    ll.RobotStart();
    // Simple always on signal to verify logging is working in AdvantageScope.
    Logger.recordOutput("Robot/Heartbeat", Timer.getFPGATimestamp());
    Logger.recordOutput("Robot/CurrentPose", DriveSubsystem.getDrivetrain().getState().Pose);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    m_robotContainer.updateRisen();
    AimUtil.updateShooterConstants();
    CommandScheduler.getInstance().run();

    Logger.recordOutput("GameHelpers/matchTimeLeft", GameHelpers.matchTimeLeft());
    Logger.recordOutput("GameHelpers/scoringTimeLeft", GameHelpers.scoringTimeLeft());
    ll.RobotEnd(isEnabled());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (hasRunTeleop) {
      LoggingInitializer.getInstance().stopLogging();
    }
  
    if (autoHoncho != null) {
      autoHoncho.stopStateMachine();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    GameHelpers.zeroTimer();
    GameHelpers.initializeStartNumber();
    autoHoncho = new AutoHoncho();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    ll.AutonomousStart();
    ll.AutonomousEnd();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (autoHoncho != null) {
      autoHoncho.stopStateMachine();
    }

    hasRunTeleop = true;
    GameHelpers.zeroTimer();
    GameHelpers.initializeStartNumber();
    DriveSubsystem.getInstance().driverControl();
    IntakeSubsystem.getInstance().stopIntake();
    IntakeSubsystem.getInstance().deployIntake();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    ll.TeleopStart();
    ll.TeleopEnd();
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
