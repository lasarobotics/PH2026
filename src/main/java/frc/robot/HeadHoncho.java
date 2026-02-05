package frc.robot;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class HeadHoncho extends StateMachine implements AutoCloseable {
  
  public enum HeadHonchoStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        ShooterSubsystem.getInstance().stopEverything();
        IntakeSubsystem.getInstance().stopIntake();
        DriveSubsystem.getInstance().driverControl();
        ClimbSubsystem.getInstance().stow();
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    },
    NORMAL {
      @Override
      public void initialize() {
        ClimbSubsystem.getInstance().stow();
      }

      @Override
      public void execute() {
        boolean shootButton = s_headHoncho.m_shootButton.getAsBoolean();
        boolean passButton = s_headHoncho.m_passButton.getAsBoolean();

        ShooterSubsystem.getInstance().shooterPeriodic(
          shootButton, passButton
        );

        if (shootButton || passButton) {
          DriveSubsystem.getInstance().driveAutoAim();
        } else {
          DriveSubsystem.getInstance().driverControl();
        }

        if (s_headHoncho.m_intakeButtonDown.getAsBoolean()) {
          IntakeSubsystem.getInstance().toggleIntake();
        }
      }

      @Override
      public SystemState nextState() {
        if (s_headHoncho.m_climbButton.getAsBoolean()) return CLIMB;

        return this;
      }
    },
    CLIMB {
      // assume true when starting state
      boolean climbButtonWasDown = true;

      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
        ShooterSubsystem.getInstance().stopEverything();
        DriveSubsystem.getInstance().driveAutoClimb();
        ClimbSubsystem.getInstance().stow();
      }

      @Override
      public void execute() {
        // TODO do the drive thing where it makes it go slow once at tower

        boolean climbButton = s_headHoncho.m_climbButton.getAsBoolean();
        // TODO also check if drivetrain is in position
        if (climbButton && !climbButtonWasDown) {
          ClimbSubsystem.getInstance().climb();
        }

        climbButtonWasDown = climbButton;
      }

      @Override
      public SystemState nextState() {
        if (s_headHoncho.m_cancelButton.getAsBoolean()) return NORMAL;

        return this;
      }
    }
  }

    private static HeadHoncho s_headHoncho;
    private BooleanSupplier m_shootButton;
    private BooleanSupplier m_passButton;
    private BooleanSupplier m_intakeButtonDown;
    private BooleanSupplier m_climbButton;
    private BooleanSupplier m_cancelButton;
    private BooleanSupplier m_goToToggle;

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  private HeadHoncho() {
    super(HeadHonchoStates.REST);
  }

    /**
     * The bindings to control the robot.
     * Should only be called once on startup.
     * @param shootButton
     * @param passButton
     * @param intakeButton
     */
    public void configureBindings(
        BooleanSupplier shootButton,
        BooleanSupplier passButton,
        BooleanSupplier intakeButton,
        BooleanSupplier climbButton,
        BooleanSupplier cancelButton
    ) {
        m_shootButton = shootButton;
        m_passButton = passButton;
        m_intakeButtonDown = intakeButton;
        m_climbButton = climbButton;
        m_cancelButton = cancelButton;
    }

  // TODO impl
  public void close() {}
}
