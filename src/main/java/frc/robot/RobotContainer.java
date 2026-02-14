// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // TODO

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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

    // TODO figure out actual bindings
    HeadHoncho.getInstance().configureBindings(
      // shoot button
      m_driverController.a(),
      // dumbshoot button
      m_driverController.b(),
      // forceshoot button
      m_driverController.b(),
      // pass button
      m_driverController.a(),
      // cancel button
      m_driverController.a(),
      // reverse intake button
      m_driverController.a(),
      // over ramp button
      () -> m_driverController.getHID().getRightTriggerAxis() > 0.5,
      // intake fallen trigger
      () -> m_driverController.getHID().getAButtonPressed(),
      // climb fallen trigger
      () -> m_driverController.getHID().getAButtonPressed(),
      // rest fallen trigger
      () -> m_driverController.getHID().getAButtonPressed()
    );


    m_driverController
        .start()
        .onTrue(
            new InstantCommand(
                    () -> {
                      // Prevent pose resets during AUTON
                      if (DriverStation.isAutonomous() && DriverStation.isEnabled()) return;
                      DriveSubsystem.getInstance().resetPoseToZero();
                    })
                .ignoringDisable(true));

    DriveSubsystem.getInstance().bindControls(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> false,
        () -> false);
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
