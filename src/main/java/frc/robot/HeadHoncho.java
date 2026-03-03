package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
        ClimbSubsystem.getInstance().stow();
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
        // ClimbSubsystem.getInstance().stow();
      }

      @Override
      public void execute() {
        boolean shoot = s_headHoncho.m_shootButton.getAsBoolean() ||
                        AutoHoncho.autoWantToShoot() ||
                        s_headHoncho.m_dumbShootButton.getAsBoolean() ||
                        AutoHoncho.autoWantToDumbShoot();

        Translation3d shootPos = wantedShootPosition();
        AimUtil.setTarget(
          new Translation2d(shootPos.getX(), shootPos.getY()),
          shootPos.getZ()
        );

        if (shoot) {
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
        if (s_headHoncho.m_overRampButton.getAsBoolean()) {
          ClimbSubsystem.getInstance().setClimbServo();
        } else {
          ClimbSubsystem.getInstance().setClimbServoZero();
        }
      }

      @Override
      public SystemState nextState() {
        // if (s_headHoncho.m_overRampButton.getAsBoolean())
        //   return OVER_RAMP;

        if (
          s_headHoncho.m_climbButtonHasFallen.getAsBoolean()
        ) return CLIMB;

        // if (s_headHoncho.m_restButtonHasFallen.getAsBoolean()) return REST;

        // return this;
        return NORMAL;
      }
    },
    OVER_RAMP {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().driveOverRamp();
      }

      @Override
      public SystemState nextState() {
        if (!s_headHoncho.m_overRampButton.getAsBoolean()) return NORMAL;

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
        //DriveSubsystem.getInstance().driveAutoClimb();
        ClimbSubsystem.getInstance().deploy();
      }

      @Override
      public void execute() {
        if (s_headHoncho.m_climbButtonHasFallen.getAsBoolean() && ClimbSubsystem.getInstance().inDeployPosition()) {
          Logger.recordOutput("HeadHoncho/executeClimbButtonReady", true);
          ClimbSubsystem.getInstance().climb();
        } else {
          Logger.recordOutput("HeadHoncho/executeClimbButtonReady", false);
        }
      }

      @Override
      public SystemState nextState() {
        if (s_headHoncho.m_cancelButton.getAsBoolean()) return UNCLIMB;

        return this;
      }
    },
    UNCLIMB {
      @Override
      public void initialize() {
        ClimbSubsystem.getInstance().deploy();
      }

      @Override
      public void execute() {
        if (ClimbSubsystem.getInstance().inDeployPosition()) {
          DriveSubsystem.getInstance().leaveClimb();
        }
      }

      @Override
      public SystemState nextState() {
        if (DriveSubsystem.getInstance().isPastTower() || s_headHoncho.m_cancelButton.getAsBoolean()) return NORMAL;

        return this;
      }
    }
  }

  private static HeadHoncho s_headHoncho;
  private BooleanSupplier m_shootButton;
  private BooleanSupplier m_dumbShootButton;
  private BooleanSupplier m_forceShootButton;
  private BooleanSupplier m_cancelButton;
  private BooleanSupplier m_reverseIntakeButton;
  private BooleanSupplier m_overRampButton;
  private BooleanSupplier m_intakeButtonHasFallen;
  private BooleanSupplier m_climbButtonHasFallen;
  private BooleanSupplier m_restButtonHasFallen;
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
    m_climbChooser.setDefaultOption("Blue Left", "Blue Left");
    m_climbChooser.addOption("Blue Right", "Blue Right");
    m_climbChooser.addOption("Red Left", "Red Left");
    m_climbChooser.addOption("Red Right", "Red Right");

    AutoHoncho.s_autoQuadrantChooser.setDefaultOption("Blue Left", "Blue Left");
    AutoHoncho.s_autoQuadrantChooser.addOption("Blue Right", "Blue Right");
    AutoHoncho.s_autoQuadrantChooser.addOption("Red Left", "Red Left");
    AutoHoncho.s_autoQuadrantChooser.addOption("Red Right", "Red Right");

    AutoHoncho.s_autoTypeChooser.setDefaultOption("Basic Shoot", "Basic Shoot");
    AutoHoncho.s_autoTypeChooser.addOption("Nothing", "Nothing");
    AutoHoncho.s_autoTypeChooser.addOption("Shoot and Climb", "Shoot and Climb");

    // Update DriveSubsystem when the climbChooser changes
    m_climbChooser.onChange(DriveSubsystem::setClimbPosition);
    AutoHoncho.s_autoQuadrantChooser.onChange(AutoHoncho::setAutoQuadrant);
    AutoHoncho.s_autoTypeChooser.onChange(AutoHoncho::setAutoType);
  }

  public void periodic() {
    Logger.recordOutput(getName() + "/currentState", getState().toString());
    Logger.recordOutput(getName() + "/overRampButton", m_overRampButton.getAsBoolean());
  }

  /**
   * Get the sendable chooser for climb auto options
   * @return A SendableChooser containing climb options
   */
  public SendableChooser<String> getClimbChooser() {
    return m_climbChooser;
  }

  public boolean wantToShoot() {
    return m_shootButton.getAsBoolean() || AutoHoncho.autoWantToShoot();
  }

  public boolean wantToDumbShoot() {
    return m_dumbShootButton.getAsBoolean() || AutoHoncho.autoWantToDumbShoot();
  }

  public boolean wantToForceShoot() {
    return m_forceShootButton.getAsBoolean();
  }

  public boolean wantToCancel() {
    return m_cancelButton.getAsBoolean();
  }

  public boolean wantToCrossRamp() {
    return m_overRampButton.getAsBoolean() || AutoHoncho.autoWantToCrossRamp();
  }

  /**
   * Determine and return the optimal shooting target
   * based on current robot position. If currently in alliance
   * zone, set this to hub - if not, set it to one of two positions
   * in the alliance zone (side of hub determines which point).
   * @return Translation3d representing current target shooting point.
   */
  private static Translation3d wantedShootPosition() {
    // Basically, have a target position and height
    // that are composed into a translation3d at the end
    Translation2d targetPos;
    double targetH;
    if (DriveSubsystem.inAllianceZone()) {
      // in alliance zone
      targetH = Constants.Field.HUB_Y_POS;

      if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
        // red alliance
        targetPos = Constants.Field.RED_HUB_COORDINATES;
      } else {
        // blue alliance
        targetPos = Constants.Field.BLUE_HUB_COORDINATES;
      }
    } else {
      // not in alliance zone
      // (shooting towards the ground)
      targetH = 0;

      Pose2d pose = DriveSubsystem.getDrivetrain().getState().Pose;
      // Top/bottom half of field is based on which side of the
      // hub the robot is on.
      boolean bottomHalfOfField = pose.getY() <= Constants.Field.HALF_FIELD_Y_POS;
      if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
        // red alliance
        targetPos = bottomHalfOfField ?
          Constants.Field.RED_BOTTOM_PASS_COORDINATES :
          Constants.Field.RED_TOP_PASS_COORDINATES;
      } else {
        // blue alliance
        targetPos = bottomHalfOfField ?
          Constants.Field.BLUE_BOTTOM_PASS_COORDINATES :
          Constants.Field.BLUE_TOP_PASS_COORDINATES;
      }
    }

    return new Translation3d(
      targetPos.getX(),
      targetPos.getY(),
      targetH
    );
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
    m_cancelButton = cancelButton;
    m_reverseIntakeButton = reverseIntakeButton;
    m_overRampButton = overRampRequest;
    m_resetOdometry = resetOdomButton;
    m_intakeButtonHasFallen = intakeButtonHasFallen;
    m_climbButtonHasFallen = climbButtonHasFallen;
    m_restButtonHasFallen = restButtonHasFallen;

    DriveSubsystem.getInstance().bindControls(
      driveRequest,
      strafeRequest,
      rotateRequest
    );
  }

  public void close() {}
}
