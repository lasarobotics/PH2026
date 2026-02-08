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

        if (shootButton || passButton) {
          DriveSubsystem.getInstance().driveAutoAim();
        } else {
          DriveSubsystem.getInstance().driverControl();
        }

        if (s_headHoncho.m_intakeButtonHasFallen.getAsBoolean()) {
          IntakeSubsystem.getInstance().toggleIntake();
        }
      }

      @Override
      public SystemState nextState() {
        if (
          s_headHoncho.m_climbButtonHasFallen.getAsBoolean() &&
          DriveSubsystem.getInstance().inAllianceZone()
        ) return CLIMB;

        return this;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        // TODO should the intake be stopped? since that means retracting hopper
        // or does it need to actively empty it if full
        IntakeSubsystem.getInstance().startIntake();
        ShooterSubsystem.getInstance().stopEverything();
        DriveSubsystem.getInstance().driveAutoClimb();
        ClimbSubsystem.getInstance().stow();
      }

      @Override
      public void execute() {
        if (s_headHoncho.m_climbButtonHasFallen.getAsBoolean()) {
          ClimbSubsystem.getInstance().climb();
        }
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
  private BooleanSupplier m_dumbShootButton;
  private BooleanSupplier m_forceShootButton;
  private BooleanSupplier m_passButton;
  private BooleanSupplier m_intakeButtonHasFallen;
  private BooleanSupplier m_climbButtonHasFallen;
  private BooleanSupplier m_cancelButton;
  private BooleanSupplier m_goToToggle;

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  private HeadHoncho() {
    super(HeadHonchoStates.NORMAL);
  }

  public boolean wantToShoot() {
    return m_shootButton.getAsBoolean();
  }

  public boolean wantToPass() {
    return m_passButton.getAsBoolean();
  }

  public boolean wantToDumbShoot() {
    return m_dumbShootButton.getAsBoolean();
  }

  public boolean wantToForceShoot() {
    return m_forceShootButton.getAsBoolean();
  }

  /**
   * The bindings to control the robot.
   * Should only be called once on startup.
   * @param shootButton True if we currently want to shoot.
   * @param dumbShootButton True if we currently want to
   * do a dumb shoot (constant hood position & speed).
   * @param forceShootButton True if we want to override
   * shooter checks & force run the indexer.
   * @param passButton True if we currently want to pass.
   * @param intakeButton True if we want to toggle the intake.
   * As a result, should be a provider for if the button has
   * just been pressed down. (e.g. () -> button.wasPressed())
   * @param climbButton True if we want to do next climb action.
   * Same as intakeButton; should be a provider for if button
   * has just been pressed.
   * @param cancelButton True if we want to go back from the climb
   * to the normal state in HeadHoncho.
   */
  public void configureBindings(
    BooleanSupplier shootButton,
    BooleanSupplier dumbShootButton,
    BooleanSupplier forceShootButton,
    BooleanSupplier passButton,
    BooleanSupplier intakeButton,
    BooleanSupplier climbButton,
    BooleanSupplier cancelButton
  ) {
    m_shootButton = shootButton;
    m_dumbShootButton = dumbShootButton;
    m_forceShootButton = forceShootButton;
    m_passButton = passButton;
    m_intakeButtonHasFallen = intakeButton;
    m_climbButtonHasFallen = climbButton;
    m_cancelButton = cancelButton;
  }

  public void close() {}
}
