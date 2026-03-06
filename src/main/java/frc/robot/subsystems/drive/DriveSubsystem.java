package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AimUtil;
import frc.robot.Constants;
import frc.robot.HeadHoncho;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

  public enum DriveStates implements SystemState {
    NOTHING {
      @Override
      public SystemState nextState() {
        return this;
      }
    },
    DISABLED {
      @Override
      public void initialize() {
        s_shouldDoGlobalPoseEstimation = true;

        // all limelights idle
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
        );

        getInstance().setAllLimelightsToAllTags();
      }

      @Override
      public void execute() {
        // LimelightHelpers.PoseEstimate pose_estimate =
        //   getInstance().getFilteredLimelightPose(Constants.Drive.SHOOTER_LIMELIGHT_NAME);
        
        // if (pose_estimate != null) {
        //   s_drivetrain.resetPose(pose_estimate.pose);
        // }
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return this;
        if (DriverStation.isAutonomous()) return AUTO;
        if (DriverStation.isTeleop()) return DRIVER_CONTROL;

        return this;
      }
    },
    AUTO {
      @Override
      public void initialize() {
        // turn on climb and shoot at full, back at idle
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = true;
        
        s_driveSubsystem.setAllLimelightsToAllTags();
      }

      @Override 
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        if (!DriverStation.isAutonomous()) return DRIVER_CONTROL;

        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        // if (s_requestedDriveState == DriveStates.CLIMB_ALIGN && inAllianceZone()) return CLIMB_ALIGN;

        return this;
      }
    },
    DRIVER_CONTROL {
      @Override
      public void initialize() {
        // turn on all at a throttle
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
        );
        s_shouldDoGlobalPoseEstimation = true;
        
        s_driveSubsystem.setAllLimelightsToAllTags();
      }

      @Override
      public void execute() {
        s_drivetrain.setControl(
          s_drive
            .withVelocityX(
              Constants.Drive.MAX_SPEED
                .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                .times(Constants.Drive.FAST_SPEED_SCALAR))
            .withVelocityY(
              Constants.Drive.MAX_SPEED
                .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
                .times(Constants.Drive.FAST_SPEED_SCALAR))
            .withRotationalRate(
              Constants.Drive.MAX_ANGULAR_RATE
                .times(-s_rotateRequest.getAsDouble())
                .times(Constants.Drive.FAST_SPEED_SCALAR)));

      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        if (DriverStation.isAutonomous()) return AUTO;

        if (s_requestedDriveState == DriveStates.OVER_RAMP) {
          return OVER_RAMP;
        }

        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        if (s_requestedDriveState == DriveStates.CLIMB_ALIGN && inAllianceZone()) return CLIMB_ALIGN;
        
        return this;
      }
    },
    AUTO_AIM {
      @Override
      public void initialize() {
        // turn on shooter limelight
        // turn off back & climb
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );

        if (!inAllianceZone()) {
          // passing
          LimelightHelpers.SetThrottle(
            Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
          );
          s_shouldDoGlobalPoseEstimation = true;

          s_driveSubsystem.setAllLimelightsToAllTags();
        } else {
          // in alliance zone, need accurate position
          // ergo local
          LimelightHelpers.SetThrottle(
            Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
          );
          s_shouldDoGlobalPoseEstimation = false;

          if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
            // red alliance
            LimelightHelpers.SetFiducialIDFiltersOverride(
              Constants.Drive.SHOOTER_LIMELIGHT_NAME,
              Constants.Drive.RED_HUB_APRIL_TAGS
            );
          } else {
            // blue alliance
            LimelightHelpers.SetFiducialIDFiltersOverride(
              Constants.Drive.SHOOTER_LIMELIGHT_NAME,
              Constants.Drive.BLUE_HUB_APRIL_TAGS
            );
          }
        }
      }

      @Override
      public void execute() {
        LimelightHelpers.PoseEstimate pose_estimate =
          getInstance().getFilteredLimelightPose(Constants.Drive.SHOOTER_LIMELIGHT_NAME);
        
        if (pose_estimate != null) {
          s_drivetrain.resetPose(pose_estimate.pose);
        }

        double currentAngle = s_drivetrain.getState().Pose.getRotation().getRadians();
        double angle = AimUtil.getRobotHeading().in(Radians);
        double output = s_autoAimController.calculate(currentAngle, angle);

        s_drivetrain.setControl(
          s_drive
            .withVelocityX(
              Constants.Drive.MAX_SPEED
                .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                .times(Constants.Drive.FAST_SPEED_SCALAR))
            .withVelocityY(
              Constants.Drive.MAX_SPEED
                .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
                .times(Constants.Drive.FAST_SPEED_SCALAR))
            .withRotationalRate(
              output
            ));
      }

      @Override
      public void end(boolean interrupted) {
        LimelightHelpers.SetFiducialIDFiltersOverride(
          Constants.Drive.CLIMB_LIMELIGHT_NAME,
          Constants.Drive.ALL_APRIL_TAGS
        );
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        if (s_requestedDriveState == DriveStates.OVER_RAMP) {
          return OVER_RAMP;
        }

        if (s_requestedDriveState == DriveStates.DRIVER_CONTROL) return DRIVER_CONTROL;
        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        if (s_requestedDriveState == DriveStates.CLIMB_ALIGN && inAllianceZone()) return CLIMB_ALIGN;

        return this;
      }
    },
    CLIMB_ALIGN {
      @Override
      public void initialize() {
        s_climbAlignSpeed = Constants.Drive.MAX_SPEED;
        s_climbRotationSpeed = Constants.Drive.MAX_ANGULAR_RATE;
        s_climbAlignDistanceError = Meters.of(0.2);
        s_climbAlignRotationError = Radians.of(0.1);

        // turn on climb limelight
        // turn off back & shooter
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = false;

        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
          // red alliance
          LimelightHelpers.SetFiducialIDFiltersOverride(
            Constants.Drive.CLIMB_LIMELIGHT_NAME,
            Constants.Drive.RED_TOWER_AND_HUB_APRIL_TAGS
          );
        } else {
          // blue alliance
          LimelightHelpers.SetFiducialIDFiltersOverride(
            Constants.Drive.CLIMB_LIMELIGHT_NAME,
            Constants.Drive.BLUE_TOWER_AND_HUB_APRIL_TAGS
          );
        }
      }

      @Override
      public void execute() {
        LimelightHelpers.PoseEstimate pose_estimate =
          getInstance().getFilteredLimelightPose(Constants.Drive.SHOOTER_LIMELIGHT_NAME);
        
        if (pose_estimate != null) {
          s_drivetrain.resetPose(pose_estimate.pose);
        }

        s_driveSubsystem.goTo(
          s_climbPosition,
          MetersPerSecond.zero(),
          DriveSubsystem.s_climbAlignSpeed,
          DriveSubsystem.s_climbRotationSpeed
        );
      }

      @Override
      public void end(boolean interrupted) {
        LimelightHelpers.SetFiducialIDFiltersOverride(
          Constants.Drive.CLIMB_LIMELIGHT_NAME,
          Constants.Drive.ALL_APRIL_TAGS
        );
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        if (s_requestedDriveState == DriveStates.OVER_RAMP) {
          return OVER_RAMP;
        }

        if (
          atDestination(
            s_climbPosition,
            DriveSubsystem.s_climbAlignDistanceError,
            DriveSubsystem.s_climbAlignRotationError
          )
        ) {
          s_climbAlignSpeed = Constants.Drive.MAX_SPEED.div(4);
          s_climbRotationSpeed = Constants.Drive.MAX_ANGULAR_RATE.div(2);
          s_climbAlignDistanceError = Meters.of(0.01);
          s_climbAlignRotationError = Radians.of(0.01);

          if (
            atDestination(
              s_climbPosition,
              DriveSubsystem.s_climbAlignDistanceError,
              DriveSubsystem.s_climbAlignRotationError
            )
          ) {          
            return 
              DriverStation.isAutonomous() ?
                AUTO :
                SLOW_DRIVER_ALIGN;
          }
        }

        // To maintain complete driver control, potentially delete though
        if (s_requestedDriveState == DriveStates.DRIVER_CONTROL) return DRIVER_CONTROL;
        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;

        return this;
      }
    }, 
    SLOW_DRIVER_ALIGN {
      @Override
      public void initialize() {
        // only turn on climb limelight
        // turn off the others
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = true;

        s_driveSubsystem.setAllLimelightsToAllTags();
      }

      @Override
      public void execute() {
        LimelightHelpers.PoseEstimate pose_estimate =
          getInstance().getFilteredLimelightPose(Constants.Drive.SHOOTER_LIMELIGHT_NAME);
        
        if (pose_estimate != null) {
          s_drivetrain.resetPose(pose_estimate.pose);
        }

        s_drivetrain.setControl(
          s_drive
            .withVelocityX(
              Constants.Drive.MAX_SPEED
                .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                .times(Constants.Drive.FAST_SPEED_SCALAR))
            .withVelocityY(
              Constants.Drive.MAX_SPEED
                .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
                .times(Constants.Drive.FAST_SPEED_SCALAR))
            .withRotationalRate(
              Constants.Drive.MAX_ANGULAR_RATE
                .times(-s_rotateRequest.getAsDouble())
                .times(Constants.Drive.FAST_SPEED_SCALAR)));
      }

      @Override
      public DriveStates nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        if (s_requestedDriveState == DriveStates.DRIVER_CONTROL) return DRIVER_CONTROL;
        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        if (s_requestedDriveState == DriveStates.CLIMB_ALIGN && inAllianceZone()) return CLIMB_ALIGN;

        return this;
      }
    },
    OVER_RAMP {
      private Pose2d[] rampSequence;
      private Pose2d currentTarget;
      private int sequenceIndex;
      private double directionSign;
      private boolean finished; // (AI Fix)

      @Override
      public void initialize() {
        sequenceIndex = 0;
        finished = false; // (AI Fix)
        rampSequence = new Pose2d[3];

        Translation2d[] translations = getInstance().getBestRampSequence();
        double robotAngle = s_drivetrain.getState().Pose.getRotation().getDegrees();
        // we want to get the nearest diagonal rotation
        // instead of having a constant one
        // because snapping around 180 degrees for no reason sucks
        Rotation2d snappedAngle = Rotation2d.fromDegrees(
          Math.round((robotAngle - 45.0) / 90.0) * 90.0 + 45.0
        );

        for (int i = 0; i < 3; i++) {
          rampSequence[i] =
            new Pose2d(
              translations[i],
              snappedAngle
            );
        }

        currentTarget = rampSequence[0];

        // for line-crossing:
        // if x of final > x of start (sign = 1.0)
        // then we want the x of the robot to be greater than the
        // x of the current target to count as "passed"
        // (and vice versa).
        // this number should never be 0
        directionSign = Math.signum(rampSequence[2].getX() - rampSequence[0].getX());

        // turn on all limelights
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = true;

        s_driveSubsystem.setAllLimelightsToAllTags();

        Logger.recordOutput("DriveSubsystem/currentRampSequence", rampSequence);
      }

      @Override
      public void execute() {
        // (AI Fix)
        if (finished) {
          // Already done — hold position at zero velocity
          getInstance().stopMoving();
          return;
        }

        // if sign of current vs target is the same as final vs start
        // (i.e. same direction)
        // or if at exact same point (want to avoid getting stuck)
        // then increment
        double crossedSign = Math.signum(
          s_drivetrain.getState().Pose.getX() - currentTarget.getX()
        );
        if (
          directionSign == crossedSign ||
          crossedSign == 0
        ) {
          sequenceIndex++;
        }

        Logger.recordOutput("DriveSubsystem/sequenceIndex", sequenceIndex);

        if (sequenceIndex > 2) {
          // when all 3 ramp pos are done, stop the robot
          finished = true;
          getInstance().stopMoving();
          // leave execute() so we do not call goTo again
          return;
        }

        currentTarget = rampSequence[sequenceIndex];

        LinearVelocity maxSpeed =
          Constants.Drive.OVER_RAMP_STAGE_MAX_SPEED[sequenceIndex];
        // if last waypoint, set target speed to 0.5
        LinearVelocity targetSpeed =
          sequenceIndex == 2 ?
            MetersPerSecond.of(0.5) :
            maxSpeed;

        getInstance().goTo(
          currentTarget,
          targetSpeed,
          maxSpeed,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        // driver must hold ramp button throughout the ramp
        // if driver lets go of ramp button goes back to driver_control
        if (!HeadHoncho.getInstance().wantToCrossRamp() || HeadHoncho.getInstance().wantToCancel()) { // allow driver to go back to drive state if let go
          s_requestedDriveState = DriveStates.DRIVER_CONTROL;
          return 
            DriverStation.isAutonomous() ?
              AUTO :
              DRIVER_CONTROL;
        }
        if (finished) { // once it reaches last "stage" of ramp it goes back
          s_requestedDriveState = DriveStates.DRIVER_CONTROL;
          return 
            DriverStation.isAutonomous() ?
              AUTO :
              DRIVER_CONTROL;
        }

        return this;
      }
    }
  }

  private static DriveSubsystem s_driveSubsystem;
  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;
  // Separate FieldCentric request for goTo() — uses BlueAlliance perspective
  // so field-frame velocities are interpreted correctly on both alliances
  private static SwerveRequest.FieldCentric s_autoDrive;

  private static Pose2d s_limelightPose;
  private static double s_limelightPoseTimeStamp;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private static final Pose2d[] redPoses = new Pose2d[]{Constants.Field.RED_TOWER};
  private static final Pose2d[] bluePoses = new Pose2d[]{Constants.Field.BLUE_TOWER};

  // TODO: add more when you get more WP destinations
  private static final int WP_CLIMB = 0;

  private static final Double DEADBAND_SCALAR = 0.085;
  private static final Double ROTATIONAL_DEADBAND_SCALAR = 0.1;
  private static final Double AUTO_DEADBAND_SCALAR = 0.02;
  private static final Double AUTO_ROTATIONAL_DEADBAND_SCALAR = 0.02;

  // Distance threshold below which we remove the velocity floor
  // to allow the robot to actually settle at the target
  private static final double GOTO_SETTLE_DISTANCE = 0.15; // meters

  private boolean m_hasAppliedOperatorPerspective = false;

  private static LinearVelocity s_climbAlignSpeed = Constants.Drive.MAX_SPEED;
  private static AngularVelocity s_climbRotationSpeed = Constants.Drive.MAX_ANGULAR_RATE;
  private static Distance s_climbAlignDistanceError = Meters.of(0.2);
  private static Angle s_climbAlignRotationError = Radians.of(0.1);

  private static PIDController s_headingController;
  private static PIDController s_autoAimController;
  private static PIDController s_autoDriveController;

  private static DriveStates s_requestedDriveState;

  private static Pose2d[] s_alliancePoses;

  public static Pose2d s_climbPosition;
  public static Pose2d s_climbAlignPosition;

  //camera vars
  protected final Thread m_limelight_thread;

  private static Map<String, Boolean> s_limelightSeesTag;
  private static Map<String, Double> s_limelightHeartbeat;

  // volatile because thread safety or something
  // idk my friend andrew told me to
  private static volatile boolean s_shouldDoGlobalPoseEstimation = true;

  public static DriveSubsystem getInstance() {
    if (s_driveSubsystem == null) {
      s_driveSubsystem = new DriveSubsystem();
    }
    return s_driveSubsystem;
  }

  public static CommandSwerveDrivetrain getDrivetrain() {
    if (s_drivetrain == null) {
      // shouldn't happen
      s_drivetrain = TunerConstants.createDrivetrain();
    }
    return s_drivetrain;
  }

  private DriveSubsystem() {
    super(DriveStates.DISABLED);
    s_requestedDriveState = DriveStates.DISABLED;
    
    s_limelightSeesTag = new HashMap<>();
    s_limelightHeartbeat = new HashMap<>();

    // init sees tag just to be safe
    s_limelightSeesTag.put(Constants.Drive.SHOOTER_LIMELIGHT_NAME, false);
    s_limelightSeesTag.put(Constants.Drive.CLIMB_LIMELIGHT_NAME, false);

    // init heartbeat to avoid crash on enable
    s_limelightHeartbeat.put(Constants.Drive.SHOOTER_LIMELIGHT_NAME, 0.0);
    s_limelightHeartbeat.put(Constants.Drive.CLIMB_LIMELIGHT_NAME, 0.0);

    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      s_alliancePoses = redPoses;
    } else {
      s_alliancePoses = bluePoses;
    }

    Logger.recordOutput("DriveSubsystem/percieved_alliance", DriverStation.getAlliance().toString());

    s_drivetrain = TunerConstants.createDrivetrain();

    // Operator-perspective drive request for teleop (driver sticks)
    s_drive =
      new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
        .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    // ai slop todo remove
    // Blue-alliance-perspective drive request for goTo() autonomous driving
    // This ensures field-frame velocities computed from odometry (blue-origin)
    // are interpreted correctly regardless of which alliance we're on.
    // s_autoGoTo =
    //   new SwerveRequest.FieldCentric()
    //     .withDeadband(0)
    //     .withRotationalDeadband(0)
    //     .withDriveRequestType(DriveRequestType.Velocity)
    //     .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    //     .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        
    s_autoDrive =
      new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.AUTO_DEADBAND_SCALAR))
        .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(DriveSubsystem.AUTO_ROTATIONAL_DEADBAND_SCALAR))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
    s_headingController = new PIDController(3, 0.0, 0.0);
    s_headingController.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Fix this with real values
    // also add tolerance
    s_autoAimController = new PIDController(5, 0.0, 0.0);
    s_autoAimController.enableContinuousInput(-Math.PI, Math.PI);

    s_autoDriveController = new PIDController(1.75, 0.0, 0.0);
    s_autoDriveController.setTolerance(0.05); // (AI Fix) 5cm position tolerance

    // TODO: Initialize the climb position to left
    s_climbPosition = new Pose2d();

    // set throttle to idle here
    LimelightHelpers.SetThrottle(
      Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
    );
    LimelightHelpers.SetThrottle(
      Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
    );

    //init limelight thread
    m_limelight_thread = new Thread(this::limelight_thread_func);
    m_limelight_thread.setDaemon(true);
    m_limelight_thread.start();
    
    Logger.recordOutput("DriveSubsystem/Odometry/target", new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  private LimelightHelpers.PoseEstimate getFilteredLimelightPose(String limelight) {
    LimelightHelpers.PoseEstimate pose_estimate =
      LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

    double hb = LimelightHelpers.getHeartbeat(limelight);
    
    Double savedHb = s_limelightHeartbeat.get(limelight);
    if (
      savedHb == null ||
      s_limelightHeartbeat.get(limelight) == hb
    ) {
      return null;
    } else {
      s_limelightHeartbeat.put(
        limelight,
        hb
      );
    }

    if (pose_estimate == null) {
      return null;
    }

    if (Math.abs(s_drivetrain.getState().Speeds.omegaRadiansPerSecond) > 2 * Math.PI) {
      return null;
    }

    if (pose_estimate.tagCount == 0) {
      return null;
    }

    if (Double.isNaN(pose_estimate.pose.getX()) || Double.isNaN(pose_estimate.pose.getY()) || Double.isNaN(pose_estimate.pose.getRotation().getDegrees())) {
      return null;
    }

    return pose_estimate;
  }

  /**
   * Function to set up the LimeLights on the robot 
   */
  public void limelight_thread_func() {

    String[] limelights = {
      Constants.Drive.SHOOTER_LIMELIGHT_NAME,
      Constants.Drive.CLIMB_LIMELIGHT_NAME
    };

    while (true) {
      // (AI Fix) Re-check state each iteration so std devs update
      // as the robot transitions between DISABLED and enabled states
      if (getState() == DriveStates.DISABLED) {
        // trust limelights for rotation if disabled
        // also trust them a little less overall
        s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 1));
      } else {
        // when running, ignore limelight rotation entirely
        s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      }

      for (String limelight : limelights) {
        if (!s_shouldDoGlobalPoseEstimation) {
          continue;
        }

        LimelightHelpers.PoseEstimate pose_estimate_maybe =
          getFilteredLimelightPose(limelight);

        if (pose_estimate_maybe == null) {
          s_limelightSeesTag.put(
            limelight,
            false
          );

          continue;
        }

        LimelightHelpers.PoseEstimate pose_estimate = pose_estimate_maybe;

        s_drivetrain.addVisionMeasurement(
          pose_estimate.pose, Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds)
        );

        s_limelightSeesTag.put(
          limelight,
          true
        );
      }
      try {
        Thread.sleep(15);
      } catch (InterruptedException e) {}
    }
  }

  private void setAllLimelightsToAllTags() {
    LimelightHelpers.SetFiducialIDFiltersOverride(
      Constants.Drive.SHOOTER_LIMELIGHT_NAME,
      Constants.Drive.ALL_APRIL_TAGS
    );
    LimelightHelpers.SetFiducialIDFiltersOverride(
      Constants.Drive.CLIMB_LIMELIGHT_NAME,
      Constants.Drive.ALL_APRIL_TAGS
    );
  }

  public void bindControls(
    DoubleSupplier driveRequest,
    DoubleSupplier strafeRequest,
    DoubleSupplier rotateRequest
  ) {
    s_driveRequest = driveRequest;
    s_strafeRequest = strafeRequest;
    s_rotateRequest = rotateRequest;
  }

  /**
   * Auto-aligns to a specific target (in other words, goes to a specific target).
   * Uses a separate BlueAlliance-perspective FieldCentric request so that
   * field-frame velocities are correct on both alliances.
   * @param target The target Pose2d (in WPILib blue-origin field coordinates)
   * @param exitVelocity The target speed of the robot once done
   * @param maxVelocity max velocity the robot can go
   * @param maxRotationRate max rate of rotation the robot can rotate at
   */
  public void goTo(
    Pose2d target,
    LinearVelocity exitVelocity,
    LinearVelocity maxVelocity,
    AngularVelocity maxRotationRate
  ) {
    Logger.recordOutput("DriveSubsystem/Odometry/target", target);
    Logger.recordOutput("DriveSubsystem/Odometry/lastGotoTimestamp", Timer.getFPGATimestamp());

    Pose2d robotPose = s_drivetrain.getState().Pose;
    Translation2d newPosition = target.getTranslation().minus(robotPose.getTranslation());

    double distance = robotPose.getTranslation().getDistance(target.getTranslation());

    Logger.recordOutput("DriveSubsystem/Odometry/distance", distance);

    var directionOfTravel = newPosition.getAngle();

    Logger.recordOutput("DriveSubsystem/Odometry/directionOfTravel", directionOfTravel);

    // (AI Fix) Proper velocity calculation ---
    // The PID is trying to drive `distance` to 0, so its output is negative.
    // We negate it to get a positive "drive toward target" speed.
    double rawDriveOutput = -s_autoDriveController.calculate(distance, 0.0);

    // (AI Fix) Only add the velocity floor when far enough from the target
    // so the robot can actually settle when close.
    double velocityFloor = distance > GOTO_SETTLE_DISTANCE ? 0.2 : 0.0;

    // (AI Fix) Clamp to [0, maxVelocity] — never drive backwards away from target
    var outputVelocity = MathUtil.clamp(
      rawDriveOutput + velocityFloor + exitVelocity.in(MetersPerSecond),
      0.0,
      maxVelocity.in(MetersPerSecond)
    );

    // (AI Fix) Clamp rotation rate symmetrically ---
    double maxRotRad = maxRotationRate.in(RadiansPerSecond);
    var rotationRate = MathUtil.clamp(
      s_headingController.calculate(robotPose.getRotation().getRadians(), target.getRotation().getRadians()),
      -maxRotRad,
      maxRotRad
    );

    var xComponent = outputVelocity * directionOfTravel.getCos();
    var yComponent = outputVelocity * directionOfTravel.getSin();

    Logger.recordOutput("DriveSubsystem/Odometry/outputVelocity", outputVelocity);
    Logger.recordOutput("DriveSubsystem/Odometry/rotationRate", rotationRate);

    // (AI Fix) Use s_autoGoTo (BlueAlliance perspective) instead of s_drive ---
    // goTo() computes velocities in the WPILib blue-origin field frame.
    // s_drive uses OperatorPerspective which flips 180° on Red alliance,
    // causing the robot to drive in the wrong direction.
    s_drivetrain.setControl(
      s_autoDrive
        .withVelocityX(MetersPerSecond.of(xComponent))
        .withVelocityY(MetersPerSecond.of(yComponent))
        .withRotationalRate(rotationRate)
    );
  
    // (AI Fix) Use angle wrapping for logged heading error ---
    Logger.recordOutput("DriveSubsystem/Odometry/radiansToRotate",
      Math.abs(MathUtil.angleModulus(robotPose.getRotation().getRadians() - target.getRotation().getRadians())));
  }

  public void stopMoving() {
    s_drivetrain.setControl(
      s_autoDrive
        .withVelocityX(MetersPerSecond.of(0.0))
        .withVelocityY(MetersPerSecond.of(0.0))
        .withRotationalRate(0.0)
    );
  }

  @Override
  public void periodic() {
    boolean resettingOdom = HeadHoncho.getInstance().wantToResetOdometry();

    if (resettingOdom) {
      zeroOdometry();
    }
    
    Logger.recordOutput(getName() + "/limelightShooterTemperature", LimelightHelpers.getLatestResults(Constants.Drive.SHOOTER_LIMELIGHT_NAME).hardware.temperature);
    Logger.recordOutput(getName() + "/limelightClimbTemperature", LimelightHelpers.getLatestResults(Constants.Drive.CLIMB_LIMELIGHT_NAME).hardware.temperature);

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      Logger.recordOutput(getName() + "/settingOperatorPerspective", true);
      if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(Alliance.Red)) {
        Logger.recordOutput(getName() + "/setOperatorPerspective", Alliance.Red);
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      } else {
        Logger.recordOutput(getName() + "/setOperatorPerspective", Alliance.Blue);
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
      m_hasAppliedOperatorPerspective = true;
    } else {
      Logger.recordOutput(getName() + "/settingOperatorPerspective", false);
    }

    Logger.recordOutput(getName() + "/doingGlobalPoseEstimation", s_shouldDoGlobalPoseEstimation);
    Logger.recordOutput(getName() + "/percieved_alliance", DriverStation.getAlliance().toString());
    Logger.recordOutput(getName() + "/resettingOdometry", resettingOdom);
    Logger.recordOutput(getName() + "/inAllianceZone", inAllianceZone());
    Logger.recordOutput(getName() + "/subsystemState", getState().toString());
  }

  public void driveAutoAim() {
    s_requestedDriveState = DriveStates.AUTO_AIM;
  }

  public void driveAutoClimb() {
    s_requestedDriveState = DriveStates.CLIMB_ALIGN;
  }

  public void driverControl() {
    if (DriverStation.isAutonomous()) {
      s_requestedDriveState = DriveStates.AUTO;
    } else {
      s_requestedDriveState = DriveStates.DRIVER_CONTROL;
    }
  }

  public void driveOverRamp() {
    s_requestedDriveState = DriveStates.OVER_RAMP;
  }

  public void leaveClimb() {
    // TODO: Make the drive subsystem back up from tower when done climbing
  }

  public boolean isPastTower() {
    // TODO: Figure out if the drive is free of the tower
    return false;
  }

  /**
   * Checks robot's alliance and then checks if robot is in its alliance zone
   * @return true if robot is in alliance zone, false otherwise
   */
  public static boolean inAllianceZone() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) &&
      s_drivetrain.getState().Pose.getX() >= Constants.Field.RED_ZONE_X
    ) {
      return true;
    } else if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) &&
      s_drivetrain.getState().Pose.getX() <= Constants.Field.BLUE_ZONE_X
    ) {
      return true;
    }
    return false;
  }

  /**
   * @param target position robot is trying to reach
   * @param acceptableDistanceError how much error is acceptable in terms of distance to the target
   * @param acceptableRotationError how much error is acceptable in terms of angle relative to desired Pose2d
   * (compared in degrees internally — pass the Angle in whatever unit, it gets converted)
   * @return true if robot is at destination, false otherwise
   */
  public static boolean atDestination(
    Pose2d target,
    Distance acceptableDistanceError,
    Angle acceptableRotationError
  ) {
    Pose2d robotPose = s_drivetrain.getState().Pose;
    double distance = Math.abs(robotPose.getTranslation().getDistance(target.getTranslation()));
    double rotationError = robotPose.getRotation().minus(target.getRotation()).getMeasure().abs(Degrees);
    
    if (
      distance < acceptableDistanceError.in(Meters) &&
      rotationError < acceptableRotationError.in(Degrees)
    ) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atWantedRotation() {
    return AimUtil.getRobotHeading().isNear(
      s_drivetrain.getState().Pose.getRotation().getMeasure(),
      Constants.Drive.ROTATION_TOLERANCE
    );
  }

  /**
   * Resets gyro heading of robot
   */
  public void zeroOdometry() {
    Rotation2d rotation;
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      // red alliance
      rotation = CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation;
    } else {
      // blue alliance
      rotation = CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation;
    }
    s_drivetrain.seedFieldCentric(rotation);
  }

  /**
   * Set the target position for drivetrain for climbing
   * Defaults to left side if provided String does not match any climb spot
   * @param selectedValue A String of either "Left", "Right", or "Center" indicating where to climb
   */
  public static void setClimbPosition(String selectedValue) {
    Logger.recordOutput("DriveSubsystem/climbPosition", selectedValue);
    switch (selectedValue) {
      case "Blue Right":
        s_climbAlignPosition = Constants.Field.BLUE_TOWER_OUTPOST_ALIGN_POSE;
        s_climbPosition = Constants.Field.BLUE_TOWER_OUTPOST_SIDE;
        break;
      case "Blue Left":
          s_climbAlignPosition = Constants.Field.BLUE_TOWER_DEPOT_ALIGN_POSE;
        s_climbPosition = Constants.Field.BLUE_TOWER_DEPOT_SIDE;
        break;
      case "Red Right":
          s_climbAlignPosition = Constants.Field.RED_TOWER_OUTPOST_ALIGN_POSE;
        s_climbPosition = Constants.Field.RED_TOWER_OUTPOST_SIDE;
        break;
      case "Red Left":
          s_climbAlignPosition = Constants.Field.RED_TOWER_DEPOT_ALIGN_POSE;
        s_climbPosition = Constants.Field.RED_TOWER_DEPOT_SIDE;
        break;
    }
  }

  private Translation2d[] getBestRampSequence() {
    Pose2d current = s_drivetrain.getState().Pose;

    Translation2d[] azCandidates = Constants.Drive.AZ_RAMP_POSA_CANDIDATES;
    Translation2d[] nzCandidates = Constants.Drive.NZ_RAMP_POSA_CANDIDATES;

    double bestAzDistance = Double.MAX_VALUE;
    int bestAzIndex = 0;
    for (int i = 0; i < azCandidates.length; i++) {
      double dist = current.getTranslation().getDistance(azCandidates[i]);
      if (dist < bestAzDistance) {
        bestAzDistance = dist;
        bestAzIndex = i;
      }
    }

    double bestNzDistance = Double.MAX_VALUE;
    int bestNzIndex = 0;
    for (int i = 0; i < nzCandidates.length; i++) {
      double dist = current.getTranslation().getDistance(nzCandidates[i]);
      if (dist < bestNzDistance) {
        bestNzDistance = dist;
        bestNzIndex = i;
      }
    }

    boolean useNz = bestNzDistance < bestAzDistance;
    int bestIndex = useNz ? bestNzIndex : bestAzIndex;

    if (useNz) {
      switch (bestIndex) {
        case 0 -> {
          return Constants.Drive.NZ_rampRed1;
        }
        case 1 -> {
          return Constants.Drive.NZ_rampRed2;
        }
        case 2 -> {
          return Constants.Drive.NZ_rampBlue1;
        }
        case 3 -> {
          return Constants.Drive.NZ_rampBlue2;
        }
      }
    } else {
      switch (bestIndex) {
        case 0 -> {
          return Constants.Drive.AZ_rampRed1;
        }
        case 1 -> {
          return Constants.Drive.AZ_rampRed2;
        }
        case 2 -> {
          return Constants.Drive.AZ_rampBlue1;
        }
        case 3 -> {
          return Constants.Drive.AZ_rampBlue2;
        }
      }
    }

    return null;
  }

  public Pose2d getLimeLightRawPose() {
    return s_limelightPose;
  }

  public double getLimeLightTimeStampSeconds() {
    return s_limelightPoseTimeStamp;
  }

  @Override
  public void close() throws Exception {
    s_drivetrain.close();
  }

}