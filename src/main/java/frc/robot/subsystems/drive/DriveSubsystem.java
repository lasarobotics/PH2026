package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.LoopTimer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class DriveSubsystem extends StateMachine implements AutoCloseable {
    public static record Hardware() {}

    public enum DriveStates implements SystemState {
      NOTHING {

        @Override
        public SystemState nextState() {
          return this;
        }
      },
      AUTO {
        @Override 
        public SystemState nextState() {
          if (!DriverStation.isAutonomous()) return DRIVER_CONTROL;
          return this;
        }
      },
      DRIVER_CONTROL {
        @Override
        public void execute() {
          s_drivetrain.setControl(
              s_drive
                  .withVelocityX(
                      Constants.Drive.MAX_SPEED
                          .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                          .times(s_driveSpeedScalar))
                  .withVelocityY(
                      Constants.Drive.MAX_SPEED
                          .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
                          .times(s_driveSpeedScalar))
                  .withRotationalRate(
                      Constants.Drive.MAX_ANGULAR_RATE
                          .times(-s_rotateRequest.getAsDouble())
                          .times(s_driveSpeedScalar)));
        }
        @Override
        public SystemState nextState() {
          if (DriverStation.isAutonomous()) return AUTO;
          if (overBumpRisingEdge()) return OVER_BUMP;
          if (fuelAlignRisingEdge()) return FUEL_ALIGN;
          return this;
        }
      },
      AUTO_AIM {

        @Override
        public void initialize() {
          s_autoAimController.enableContinuousInput(-Math.PI, Math.PI);
          s_autoAimController.setConstraints(Constants.Drive.TURN_CONSTRAINTS);
        }

        @Override
        public void execute() {

          
        }
        @Override
        public SystemState nextState() {
          return this;
        }
      },
      CLIMB_ALIGN {
        @Override
        public SystemState nextState() {
          return this;
        }

      },
      FUEL_ALIGN {
        @Override
        public void initialize() {
          s_fuelDistanceController.reset();
        }

        @Override
        public void execute() {
          if (isIntakeFull() || !s_vision.hasValidTarget()) {
            s_hasRegisteredPickup = false;
            s_pickupDebounceFrames = 0;
            s_drivetrain.setControl(
                s_robotAlign
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
            return;
          }

          Translation2d robotTranslation = s_vision.getRobotRelativeTranslation();
          double distance = s_vision.getTargetDistance();

          if (distance < Constants.Drive.FUEL_ALIGN_STOP_DISTANCE_METERS) {
            if (!s_hasRegisteredPickup) {
              s_pickupDebounceFrames++;
              if (s_pickupDebounceFrames >= PICKUP_CONFIRM_FRAMES) {
                registerPickup();
                s_hasRegisteredPickup = true;
                s_pickupDebounceFrames = 0;
                s_vision.clearTargetLock();
              }
            }
            s_drivetrain.setControl(
                s_robotAlign
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
            return;
          }

          s_hasRegisteredPickup = false;
          s_pickupDebounceFrames = 0;

          double cappedSpeed =
              Constants.Drive.MAX_SPEED
                  .times(Constants.Drive.FUEL_ALIGN_SPEED_SCALAR)
                  .in(MetersPerSecond);

          double errorDistance = distance - Constants.Drive.FUEL_ALIGN_STOP_DISTANCE_METERS;
          if (Math.abs(errorDistance) < 0.02) {
            s_drivetrain.setControl(
                s_robotAlign
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
            return;
          }

          double pidOutput =
              s_fuelDistanceController.calculate(
                  distance, Constants.Drive.FUEL_ALIGN_STOP_DISTANCE_METERS);
          double minForward = cappedSpeed * 0.1;
          double commandedSpeed =
              Math.min(
                  Math.max(Math.max(pidOutput, 0), minForward),
                  cappedSpeed);

          double translationNorm =
              Math.max(robotTranslation.getNorm(), Constants.Vision.MIN_DISTANCE_METERS);
          Translation2d direction = robotTranslation.div(translationNorm);

          double vx = direction.getX() * commandedSpeed;
          double vy = direction.getY() * commandedSpeed;

          s_drivetrain.setControl(
              s_robotAlign
                  .withVelocityX(vx)
                  .withVelocityY(vy)
                  .withRotationalRate(0));
        }

        @Override
        public SystemState nextState() {
          if (DriverStation.isAutonomous()) return AUTO;
          if (!fuelAlignPressed()) return DRIVER_CONTROL;
          return this;
        }

      },
      OVER_BUMP {
        @Override
        public void initialize() {
          s_overBumpTimer.reset();
          s_overBumpTimer.start();
          s_overBumpPhase = OverBumpPhase.TO_POSA;
          selectClosestBumpSet();
          s_overBumpHeadingGoal = s_targetPosA.getRotation().getRadians();
          s_overBumpDistanceController.reset();
          s_overBumpHeadingController.reset();
        }

        @Override
        public void execute() {
          updateQuestPose();
          Pose2d robotPose = s_drivetrain.getState().Pose;
          switch (s_overBumpPhase) {
            case TO_POSA:
              if (driveToPose(robotPose, s_targetPosA, Constants.Drive.GO_TO_POSA_SPEED_SCALAR, OVER_BUMP_HEADING_KP, 0.1)) {
                s_overBumpPhase = OverBumpPhase.TO_POSB;
                s_overBumpHeadingGoal = s_targetPosB.getRotation().getRadians();
              }
              break;
            case TO_POSB:
              if (driveToPose(robotPose, s_targetPosB, Constants.Drive.GO_OVER_BUMP_SPEED_SCALAR, OVER_BUMP_HEADING_KP, 0.1)) {
                s_overBumpPhase = OverBumpPhase.TO_POSC;
                s_overBumpHeadingGoal = s_targetPosC.getRotation().getRadians();
              }
              break;
            case TO_POSC:
              if (driveToPose(robotPose, s_targetPosC, Constants.Drive.GO_DOWN_BUMP_SPEED_SCALAR, OVER_BUMP_HEADING_KP_FAST, 0.1)) {
                s_overBumpPhase = OverBumpPhase.DONE;
              }
              break;
            case DONE:
              s_drivetrain.setControl(
                  s_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
              break;
          }
        }

        @Override
        public void end(boolean interrupted) {
          s_overBumpTimer.stop();
        }

        @Override
        public SystemState nextState() {
          if (DriverStation.isAutonomous()) return AUTO;
          if (!overBumpPressed() || s_overBumpPhase == OverBumpPhase.DONE) return DRIVER_CONTROL;
          return this;
        }
      },
    }


  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;
  private static RobotCentricWithPose s_robotAlign;
  private static VisionSubsystem s_vision;
  private static QuestNav s_questNav;
  private static Pose3d s_latestQuestPose = new Pose3d();
  private static double s_latestQuestTimestamp = 0.0;
  private static Pose2d s_targetPosA;
  private static Pose2d s_targetPosB;
  private static Pose2d s_targetPosC;
  private static double s_lastIntakeActiveTime = Double.NEGATIVE_INFINITY;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;
  private static BooleanSupplier s_fuelAlignRequest = () -> false;
  private static BooleanSupplier s_overBumpRequest = () -> false;

  private static final Double DEADBAND_SCALAR = 0.085;

  private boolean m_hasAppliedOperatorPerspective = false;

  private static boolean s_prevFuelAlignRequest = false;
  private static boolean s_prevOverBumpRequest = false;
  private static boolean s_hasRegisteredPickup = false;
  private static double s_estimatedBallsCollected = 0.0;
  private static int s_pickupDebounceFrames = 0;

  private static double s_driveSpeedScalar = Constants.Drive.FAST_SPEED_SCALAR;

  private static ProfiledPIDController s_autoAimController;
  private static ProfiledPIDController s_climbAutoAlignController;
  private static ProfiledPIDController s_autoIntakeController;
  private static PIDController s_fuelDistanceController;
  private static PIDController s_overBumpDistanceController;
  private static PIDController s_overBumpHeadingController;
  private static final Timer s_overBumpTimer = new Timer();
  private static final double PICKUP_EFFICIENCY = 0.9;
  private static final int MAX_BALL_CAPACITY = 60;
  private static final int PICKUP_CONFIRM_FRAMES = 5;
  private static final double OVER_BUMP_DURATION_SECONDS = 1.0;
  private static final double OVER_BUMP_SPEED_SCALAR = 0.6;
  private static final double OVER_BUMP_HEADING_KP = 4.0;
  private static final double OVER_BUMP_HEADING_KP_FAST = 6.0;
  private static OverBumpPhase s_overBumpPhase = OverBumpPhase.DONE;
  private static double s_overBumpHeadingGoal = 0.0;

  public DriveSubsystem(Hardware driveHardware) {
    super(DriveStates.DRIVER_CONTROL);

    s_drivetrain = TunerConstants.createDrivetrain();
    s_vision = new VisionSubsystem();

    s_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    s_robotAlign =
        new RobotCentricWithPose()
            .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1))
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    s_autoIntakeController = new ProfiledPIDController(
      1.0, 
      1.0, 
      1.0, 
      Constants.Drive.TURN_CONSTRAINTS);
    s_fuelDistanceController = new PIDController(1.0, 0.0, 0.0);
    s_overBumpDistanceController = new PIDController(1.0, 0.0, 0.0);
    s_overBumpHeadingController = new PIDController(4.0, 0.0, 0.0);
    s_questNav = new QuestNav();
  }

  /*
   * Binds the controls needed to drive for controller usage
   */
  public void bindControls(
      DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    bindControls(driveRequest, strafeRequest, rotateRequest, () -> false, () -> false);
  }

  public void bindControls(
      DoubleSupplier driveRequest,
      DoubleSupplier strafeRequest,
      DoubleSupplier rotateRequest,
      BooleanSupplier fuelAlignRequest) {
    bindControls(driveRequest, strafeRequest, rotateRequest, fuelAlignRequest, () -> false);
  }

  public void bindControls(
      DoubleSupplier driveRequest,
      DoubleSupplier strafeRequest,
      DoubleSupplier rotateRequest,
      BooleanSupplier fuelAlignRequest,
      BooleanSupplier overBumpRequest) {
    s_driveRequest = driveRequest;
    s_strafeRequest = strafeRequest;
    s_rotateRequest = rotateRequest;
    s_fuelAlignRequest = fuelAlignRequest;
    s_overBumpRequest = overBumpRequest;
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware driveHardware = new Hardware();
    return driveHardware;
  }

  private static boolean fuelAlignPressed() {
    return s_fuelAlignRequest.getAsBoolean();
  }

  private static boolean overBumpPressed() {
    return s_overBumpRequest.getAsBoolean();
  }

  private static boolean isIntakeFull() {
    return s_estimatedBallsCollected >= MAX_BALL_CAPACITY;
  }

  private static void registerPickup() {
    s_estimatedBallsCollected = Math.min(s_estimatedBallsCollected + PICKUP_EFFICIENCY, MAX_BALL_CAPACITY);
  }

  private static boolean fuelAlignRisingEdge() {
    boolean pressed = fuelAlignPressed();
    boolean rising = pressed && !s_prevFuelAlignRequest;
    s_prevFuelAlignRequest = pressed;
    return rising;
  }

  private static boolean overBumpRisingEdge() {
    boolean pressed = overBumpPressed();
    boolean rising = pressed && !s_prevOverBumpRequest;
    s_prevOverBumpRequest = pressed;
    return rising;
  }

  @Override
  public void periodic() {

    LoopTimer.addTimestamp(getName() + " Start");
    Logger.recordOutput("Drive/EstimatedBalls", s_estimatedBallsCollected);
    Logger.recordOutput("Drive/IntakeFull", isIntakeFull());
    if (fuelAlignPressed()) {
      s_lastIntakeActiveTime = Timer.getFPGATimestamp();
    }

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      Logger.recordOutput(getName() + "/settingOperatorPerspective", true);
      if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      } else {
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
      m_hasAppliedOperatorPerspective = true;
    } else {
      Logger.recordOutput(getName() + "/settingOperatorPerspective", false);
    }
  }

  @Override
  public void close() throws Exception {
    s_drivetrain.close();
  }

  private static void updateQuestPose() {
    if (s_questNav == null) return;
    s_questNav.commandPeriodic();
    PoseFrame[] frames = s_questNav.getAllUnreadPoseFrames();
    if (frames.length > 0) {
      PoseFrame frame = frames[frames.length - 1];
      if (frame.isTracking()) {
        s_latestQuestPose = frame.questPose3d();
        s_latestQuestTimestamp = frame.dataTimestamp();
      }
    }
    Logger.recordOutput("Drive/QuestPose", s_latestQuestPose);
    Logger.recordOutput("Drive/QuestPoseTimestamp", s_latestQuestTimestamp);
  }

  public Pose3d getQuestPose() {
    return s_latestQuestPose;
  }

  public double getQuestPoseTimestamp() {
    return s_latestQuestTimestamp;
  }

  public boolean questIsTracking() {
    return s_questNav != null && s_questNav.isTracking();
  }

  private static void selectClosestBumpSet() {
    boolean allowNZ = (Timer.getFPGATimestamp() - s_lastIntakeActiveTime) <= 5.0;
    Pose2d[] posa = allowNZ
        ? new Pose2d[] {
            Constants.Drive.NZ_bumpRed1_posa,
            Constants.Drive.NZ_bumpRed2_posa,
            Constants.Drive.NZ_bumpBlue1_posa,
            Constants.Drive.NZ_bumpBlue2_posa
          }
        : new Pose2d[] {
            Constants.Drive.AZ_bumpRed1_posa,
            Constants.Drive.AZ_bumpRed2_posa,
            Constants.Drive.AZ_bumpBlue1_posa,
            Constants.Drive.AZ_bumpBlue2_posa
          };
    Pose2d[] posb = allowNZ
        ? new Pose2d[] {
            Constants.Drive.NZ_bumpRed1_posb,
            Constants.Drive.NZ_bumpRed2_posb,
            Constants.Drive.NZ_bumpBlue1_posb,
            Constants.Drive.NZ_bumpBlue2_posb
          }
        : new Pose2d[] {
            Constants.Drive.AZ_bumpRed1_posb,
            Constants.Drive.AZ_bumpRed2_posb,
            Constants.Drive.AZ_bumpBlue1_posb,
            Constants.Drive.AZ_bumpBlue2_posb
          };
    Pose2d[] posc = allowNZ
        ? new Pose2d[] {
            Constants.Drive.NZ_bumpRed1_posc,
            Constants.Drive.NZ_bumpRed2_posc,
            Constants.Drive.NZ_bumpBlue1_posc,
            Constants.Drive.NZ_bumpBlue2_posc
          }
        : new Pose2d[] {
            Constants.Drive.AZ_bumpRed1_posc,
            Constants.Drive.AZ_bumpRed2_posc,
            Constants.Drive.AZ_bumpBlue1_posc,
            Constants.Drive.AZ_bumpBlue2_posc
          };

    Pose2d robotPose = s_drivetrain.getState().Pose;
    double bestDistance = Double.POSITIVE_INFINITY;
    int bestIndex = 0;
    for (int i = 0; i < posa.length; i++) {
      double distance =
          robotPose.getTranslation().getDistance(posa[i].getTranslation());
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = i;
      }
    }

    s_targetPosA = posa[bestIndex];
    s_targetPosB = posb[bestIndex];
    s_targetPosC = posc[bestIndex];
  }

  private static boolean driveToPose(
      Pose2d currentPose,
      Pose2d targetPose,
      double speedMetersPerSecond,
      double headingKp,
      double stopDistance) {
    Translation2d targetTranslation = targetPose.getTranslation();
    Translation2d error = targetTranslation.minus(currentPose.getTranslation());
    double distance = error.getNorm();

    double targetHeading = targetPose.getRotation().getRadians();
    s_overBumpHeadingController.setP(headingKp);
    double rotationalRate =
        s_overBumpHeadingController.calculate(currentPose.getRotation().getRadians(), targetHeading);

    if (distance < stopDistance) {
      s_drivetrain.setControl(
          s_drive
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(rotationalRate));
      return true;
    }

    Translation2d direction = error.div(Math.max(distance, 1e-6));
    s_overBumpDistanceController.setP(1.0);
    double speedCommand =
        Math.min(
            Math.max(s_overBumpDistanceController.calculate(distance, 0), 0),
            speedMetersPerSecond);
    double vx = direction.getX() * speedCommand;
    double vy = direction.getY() * speedCommand;

    s_drivetrain.setControl(
        s_drive
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(rotationalRate));
    return false;
  }


  private enum OverBumpPhase {
    TO_POSA,
    TO_POSB,
    TO_POSC,
    DONE
  }
}
