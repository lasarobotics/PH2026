// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(
      Constants.SmartDashboard.SMARTDASHBOARD_CLIMB_CHOOSER_NAME, 
      HeadHoncho.getInstance().getClimbChooser()
    );
  }

  private boolean m_intakePrevious = false;
  private boolean m_climbPrevious = false;
  private boolean m_restPrevious = false;
  private boolean m_intakeRisen = false;
  private boolean m_climbRisen = false;
  private boolean m_restRisen = false;

  /**
   * Updates the rising edge detection for certain bindings (intake,
   * climb, rest). Should only be called once per loop to ensure that
   * everything that wants to know about a rising edge does.
   */
  public void updateRisen() {
    boolean intakeDown = m_driverController.leftBumper().getAsBoolean();
    boolean climbDown = m_driverController.povUp().getAsBoolean();
    boolean restDown = m_driverController.back().getAsBoolean();

    // always set to false - will be made true if there is really
    // a rising edge
    m_intakeRisen = false;
    m_climbRisen = false;
    m_restRisen = false;

    // rising edge
    if (intakeDown && !m_intakePrevious) m_intakeRisen = true;
    if (climbDown && !m_climbPrevious) m_climbRisen = true;
    if (restDown && !m_restPrevious) m_restRisen = true;

    m_intakePrevious = intakeDown;
    m_climbPrevious = climbDown;
    m_restPrevious = restDown;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    HeadHoncho.getInstance().configureBindings(
      // shoot button
      m_driverController.rightTrigger(),
      // dumbshoot button
      m_driverController.b(),
      // forceshoot button
      m_driverController.y(),
      // cancel button
      m_driverController.x(),
      // reverse intake button
      m_driverController.a(),
      // reset odom button
      m_driverController.start(),
      // over ramp button
      m_driverController.leftTrigger(),
      // intake fallen trigger
      () -> m_intakeRisen,
      // climb fallen trigger
      () -> m_climbRisen,
      // rest fallen trigger
      () -> m_restRisen,
      // drive subsystem stuff
      // drive
      () -> m_driverController.getLeftX(),
      // strafe
      () -> m_driverController.getLeftY(),
      // rotate
      () -> m_driverController.getRightX()
    );

    // this is kinda icky to have separate
    // but whatever
    m_driverController
      .start()
      .onTrue(
        new InstantCommand(
          () -> {
            // Prevent pose resets during auto
            if (DriverStation.isAutonomous() && DriverStation.isEnabled()) return;
            DriveSubsystem.getInstance().resetPoseToZero();
          }
        )
        .ignoringDisable(true)
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
