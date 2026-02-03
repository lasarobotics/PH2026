package frc.robot;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class HeadHoncho extends StateMachine implements AutoCloseable {
  
  public enum HeadHonchoStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        ShooterSubsystem.getInstance().stopEverything();
        IntakeSubsystem.getInstance().stopIntake();
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    },
    NORMAL {
      @Override
      public void execute() {
        boolean shootButton = s_headHoncho.m_shootButton.getAsBoolean();
        boolean passButton = s_headHoncho.m_passButton.getAsBoolean();

        ShooterSubsystem.getInstance().shooterPeriodic(
          shootButton, passButton
        );

        if (s_headHoncho.m_intakeButtonDown.getAsBoolean()) {
          IntakeSubsystem.getInstance().toggleIntake();
        }
      }

      // when should we transition to other states? TODO
      @Override
      public SystemState nextState() {
        return this;
      }
    },
    AUTO_INTAKE {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
        ShooterSubsystem.getInstance().stopEverything();
      }

      @Override
      public void execute() {
        // should get the wanted position from the ball targeter
        // and go there
        // TODO
      }

      // when do we want to go back? TODO
      @Override
      public SystemState nextState() {
        return this;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
        ShooterSubsystem.getInstance().stopEverything();
        
        // goto the climb position?
          // maybe the goto should be in execute
      }

      @Override
      public void execute() {
        // check if at climb position
        // move onto the tower
        // run the motor
      }

      // do we even ever want to go back?
      @Override
      public SystemState nextState() {
        return this;
      }
    }
  }

  private static HeadHoncho s_headHoncho;
  private BooleanSupplier m_shootButton;
  private BooleanSupplier m_passButton;
  private BooleanSupplier m_intakeButtonDown;

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
    BooleanSupplier intakeButton
  ) {
    m_shootButton = shootButton;
    m_passButton = passButton;
    m_intakeButtonDown = intakeButton;
  }

  // TODO impl
  public void close() {}
}
