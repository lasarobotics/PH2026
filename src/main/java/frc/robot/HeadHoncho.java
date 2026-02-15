package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class HeadHoncho extends StateMachine implements AutoCloseable {
  
  public enum HeadHonchoStates implements SystemState {
    REST {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().stopIntake();
        ShooterSubsystem.getInstance().stopOperation();
        DriveSubsystem.getInstance().driverControl();
        ClimbSubsystem.getInstance().stowClimber();
      }

      @Override
      public SystemState nextState() {
        if (s_headHoncho.m_restButtonHasFallen.getAsBoolean()) return NORMAL;

        return this;
      }
    },
    NORMAL {
      @Override
      public void initialize() {
        ShooterSubsystem.getInstance().startOperation();
        ClimbSubsystem.getInstance().stowClimber();
      }

      @Override
      public void execute() {
        boolean overRampRequested = s_headHoncho.m_overRampRequest.getAsBoolean();
        boolean shootButton = s_headHoncho.m_shootButton.getAsBoolean();
        boolean passButton = s_headHoncho.m_passButton.getAsBoolean();

        if (passButton) {
          AimUtil.setTarget(wantedPassPosition(), 0);
        } else {
          AimUtil.setTarget(
            wantedShootPosition(),
            Constants.Field.HUB_Y_POS
          );
        }

        if (!overRampRequested && (shootButton || passButton)) {
          DriveSubsystem.getInstance().driveAutoAim();
        } else {
          DriveSubsystem.getInstance().driverControl();
        }

        if (s_headHoncho.m_intakeButtonHasFallen.getAsBoolean()) {
          IntakeSubsystem.getInstance().toggleIntake();
        }

        if (s_headHoncho.m_reverseIntakeButton.getAsBoolean()) {
          IntakeSubsystem.getInstance().reverseIntake();
        }
      }

      @Override
      public SystemState nextState() {
        if (s_headHoncho.m_overRampRequest.getAsBoolean() && !DriveSubsystem.getInstance().overRampFinishedWhileHeld())
          return OVER_RAMP;

        if (
          s_headHoncho.m_climbButtonHasFallen.getAsBoolean() &&
          DriveSubsystem.getInstance().inAllianceZone()
        ) return CLIMB;

        if (s_headHoncho.m_restButtonHasFallen.getAsBoolean()) return REST;

        return this;
      }
    },
    OVER_RAMP {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().driverControl();
      }

      @Override
      public SystemState nextState() {
        if (!s_headHoncho.m_overRampRequest.getAsBoolean()) return NORMAL;
        if (DriveSubsystem.getInstance().overRampFinishedWhileHeld()) return NORMAL;
        return this;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        // The hopper can retract while full
        // it just spews balls out
        IntakeSubsystem.getInstance().stopIntake();
        ShooterSubsystem.getInstance().stopOperation();
        DriveSubsystem.getInstance().driveAutoClimb();
        ClimbSubsystem.getInstance().stowClimber();
      }

      @Override
      public void execute() {
        if (s_headHoncho.m_climbButtonHasFallen.getAsBoolean()) {
          ClimbSubsystem.getInstance().extendClimber();
        }
      }

      @Override
      public SystemState nextState() {
        if (s_headHoncho.m_overRampRequest.getAsBoolean() && !DriveSubsystem.getInstance().overRampFinishedWhileHeld())
          return OVER_RAMP;

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
  private BooleanSupplier m_cancelButton;
  private BooleanSupplier m_reverseIntakeButton;
  private BooleanSupplier m_overRampRequest;
  private BooleanSupplier m_intakeButtonHasFallen;
  private BooleanSupplier m_climbButtonHasFallen;
  private BooleanSupplier m_restButtonHasFallen;
  private BooleanSupplier m_goToToggle;
  private BooleanSupplier m_resetOdometry;

  private SendableChooser<String> m_climbChooser;

  public static HeadHoncho getInstance() {
    if (s_headHoncho == null) {
      s_headHoncho = new HeadHoncho();
    }
    return s_headHoncho;
  }

  private HeadHoncho() {
    super(HeadHonchoStates.NORMAL);

    // Set up SendableChooser to get where to climb in auto
    m_climbChooser = new SendableChooser<>();
    m_climbChooser.setDefaultOption("Left", getName());
    m_climbChooser.addOption("Center", getName());
    m_climbChooser.addOption("Right", getName());

    // Update DriveSubsystem when the climbChooser changes
    m_climbChooser.onChange(DriveSubsystem::setClimbPosition);
  }

  /**
   * Get the sendable chooser for climb auto options
   * @return A SendableChooser containing climb options
   */
  public SendableChooser<String> getClimbChooser() {
    return m_climbChooser;
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

  private static Translation2d wantedShootPosition() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      // red alliance
      return Constants.Field.RED_HUB_COORDINATES;
    } else {
      // blue alliance
      return Constants.Field.BLUE_HUB_COORDINATES;
    }
  }

  private static Translation2d wantedPassPosition() {
    Pose2d pose = DriveSubsystem.getDrivetrain().getState().Pose;
    boolean bottomHalfOfField = pose.getY() <= Constants.Field.HALF_FIELD_Y_POS;
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      // red alliance
      return bottomHalfOfField ?
        Constants.Field.RED_BOTTOM_PASS_COORDINATES :
        Constants.Field.RED_TOP_PASS_COORDINATES;
    } else {
      // blue alliance
      return bottomHalfOfField ?
        Constants.Field.BLUE_BOTTOM_PASS_COORDINATES :
        Constants.Field.BLUE_TOP_PASS_COORDINATES;
    }
  }

  public boolean wantToResetOdometry() {
    return m_resetOdometry.getAsBoolean();
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
   * @param cancelButton True if we want to go back from the climb
   * to the normal state in HeadHoncho.
   * @param reverseIntakeButton True if we want to reverse the intake motor
   * @param overRampRequest True if we want to go over the bump
   * @param restButtonHasFallen True if we want to go over the bump
   * @param intakeButtonHasFallen True if we want to toggle the intake.
   * As a result, should be a provider for if the button has
   * just been pressed down. (e.g. () -> button.wasPressed())
   * @param climbButtonHasFallen True if we want to do next climb action.
   * Same as intakeButton; should be a provider for if button
   * has just been pressed.
   * @param restButtonHasFallen True if we want to do switch between rest
   * and normal states in HeadHoncho.
   */
  public void configureBindings(
    BooleanSupplier shootButton,
    BooleanSupplier dumbShootButton,
    BooleanSupplier forceShootButton,
    BooleanSupplier passButton,
    BooleanSupplier cancelButton,
    BooleanSupplier reverseIntakeButton,
    BooleanSupplier overRampRequest,
    BooleanSupplier resetOdomButton,
    BooleanSupplier intakeButtonHasFallen,
    BooleanSupplier climbButtonHasFallen,
    BooleanSupplier restButtonHasFallen,
    DoubleSupplier driveRequest,
    DoubleSupplier strafeRequest,
    DoubleSupplier rotateRequest
  ) {
    m_shootButton = shootButton;
    m_dumbShootButton = dumbShootButton;
    m_forceShootButton = forceShootButton;
    m_passButton = passButton;
    m_cancelButton = cancelButton;
    m_reverseIntakeButton = reverseIntakeButton;
    m_overRampRequest = overRampRequest;
    m_resetOdometry = resetOdomButton;
    m_intakeButtonHasFallen = intakeButtonHasFallen;
    m_climbButtonHasFallen = climbButtonHasFallen;
    m_restButtonHasFallen = restButtonHasFallen;

    DriveSubsystem.getInstance().bindControls(
      driveRequest,
      strafeRequest,
      rotateRequest,
      overRampRequest
    );
  }

  public void close() {}
}
