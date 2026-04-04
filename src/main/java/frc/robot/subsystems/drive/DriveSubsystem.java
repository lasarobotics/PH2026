package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AimUtil;
import frc.robot.Constants;
import frc.robot.HeadHoncho;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

  public enum DriveStates implements SystemState {
    /**
     * Nothing state.
     */
    NOTHING {
      @Override
      public SystemState nextState() {
        return this;
      }
    },
    /**
     * Disabled state. Sets limelights to idle speed, all tags,
     * and enables global pose estimation.
     */
    DISABLED {
      @Override
      public void initialize() {
        s_shouldDoGlobalPoseEstimation = true;

        // all limelights idle
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.RIGHT_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
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
    /**
     * Auto state. Sets limelights to running throttle, all tags, and enables
     * global pose estimation. Has no execute to allow for AutoHoncho to utilize
     * goTo for whatever it wants to do.
     */
    AUTO {
      @Override
      public void initialize() {
        // turn on shoot at full, back at idle
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.RIGHT_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = true;
        
        s_driveSubsystem.setAllLimelightsToAllTags();
      }

      @Override 
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        if (!DriverStation.isAutonomous()) return DRIVER_CONTROL;

        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;

        return this;
      }
    },
    /**
     * Driver control (normal) state. Sets all limelights to all tags, idle throttle,
     * and enables global pose estimation. Drives robot directly based on controller
     * input.
     */
    DRIVER_CONTROL {
      @Override
      public void initialize() {
        // turn on all at a throttle
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.RIGHT_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
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
                .times(-s_strafeRequest.getAsDouble())
                .times(Constants.Drive.FAST_SPEED_SCALAR))
            .withVelocityY(
              Constants.Drive.MAX_SPEED
                .times(-s_driveRequest.getAsDouble())
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

        // if (s_requestedDriveState == DriveStates.OVER_RAMP) {
        //   return OVER_RAMP;
        // }

        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        
        return this;
      }
    },
    /**
     * Auto aim (shooting) state. If in alliance zone (shooting to hub),
     * restrict limelights to only hub tags and only use shooter limelight.
     * If passing, enable all limelights at idle and use global pose estimation
     * across all tags. Translation is directly from controller, but rotation
     * automatically aims to enable shooting to the target.
     */
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
            Constants.Drive.RIGHT_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
          );
          s_shouldDoGlobalPoseEstimation = true;

          s_driveSubsystem.setAllLimelightsToAllTags();
        } else {
          // in alliance zone, need accurate position
          // ergo local
          LimelightHelpers.SetThrottle(
            Constants.Drive.RIGHT_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
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
          s_drivetrain.resetPose(pose_estimate.pose.toPose2d());
        }

        double currentAngle = s_drivetrain.getState().Pose.getRotation().getRadians();
        double angle = AimUtil.getRobotHeading().in(Radians);
        if (
          Math.sqrt(
            Math.pow(s_drivetrain.getState().Speeds.vxMetersPerSecond, 2) +
            Math.pow(s_drivetrain.getState().Speeds.vyMetersPerSecond, 2)
          ) > 0.05
        ) {
          // moving
          s_autoAimController.setP(7);
        } else {
          // stationary
          s_autoAimController.setP(6.5);
        }
        double output = s_autoAimController.calculate(currentAngle, angle);

        Translation2d translationVec = new Translation2d(
          Constants.Drive.MAX_SPEED
            .times(-s_strafeRequest.getAsDouble())
            .times(Constants.Drive.FAST_SPEED_SCALAR).in(MetersPerSecond),
          Constants.Drive.MAX_SPEED
            .times(-s_driveRequest.getAsDouble())
            .times(Constants.Drive.FAST_SPEED_SCALAR).in(MetersPerSecond)
        );

        // 1 meter per second
        if (translationVec.getNorm() > Constants.Drive.MAX_SHOOTING_SPEED) {
          translationVec = new Translation2d(
            translationVec.getX() / translationVec.getNorm() * Constants.Drive.MAX_SHOOTING_SPEED,
            translationVec.getY() / translationVec.getNorm() * Constants.Drive.MAX_SHOOTING_SPEED
          );
        }

        s_drivetrain.setControl(
          s_autoAimDrive
            .withVelocityX(
              translationVec.getX()
            )
            .withVelocityY(
              translationVec.getY()
            )
            .withRotationalRate(
              output
            )
          );
      }

      @Override
      public void end(boolean interrupted) {
        getInstance().setAllLimelightsToAllTags();
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isDisabled()) return DISABLED;
        // if (s_requestedDriveState == DriveStates.OVER_RAMP) {
        //   return OVER_RAMP;
        // }

        if (s_requestedDriveState == DriveStates.DRIVER_CONTROL) return DRIVER_CONTROL;
        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;

        return this;
      }
    },
    /**
     * TODO nuke
     */
    OVER_RAMP {
      private Pose2d[] rampSequence;
      private Pose2d currentTarget;
      private int sequenceIndex;
      private double directionSign;
      private boolean finished;

      @Override
      public void initialize() {
        sequenceIndex = 0;
        finished = false;
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
          Constants.Drive.RIGHT_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = true;

        s_driveSubsystem.setAllLimelightsToAllTags();

        Logger.recordOutput("DriveSubsystem/currentRampSequence", rampSequence);
      }

      @Override
      public void execute() {
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
  private static SwerveRequest.FieldCentric s_autoAimDrive;
  // Separate FieldCentric request for goTo() — uses BlueAlliance perspective
  // so field-frame velocities are interpreted correctly on both alliances
  private static SwerveRequest.FieldCentric s_autoDrive;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private boolean m_hasAppliedOperatorPerspective = false;

  private static PIDController s_headingController;
  private static PIDController s_autoAimController;
  private static PIDController s_autoDriveController;

  private static DriveStates s_requestedDriveState;

  //camera vars
  protected final Thread m_limelight_thread;

  private static volatile Map<String, Double> s_limelightHeartbeat;
  private static volatile Map<String, Pose2d> s_lastGoodPose;
  private static volatile Map<String, Double> s_lastGoodPoseTime;

  // volatile because thread safety or something
  // idk my friend andrew told me to
  private static volatile boolean s_shouldDoGlobalPoseEstimation = true;

  /**
   * Get the singleton instance of DriveSubsystem
   * @return Subsystem instance
   */
  public static DriveSubsystem getInstance() {
    if (s_driveSubsystem == null) {
      s_driveSubsystem = new DriveSubsystem();
    }
    return s_driveSubsystem;
  }

  /**
   * Get the singleton instance of the drivetrain object
   * @return Drivetrain instance
   */
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
    
    s_limelightHeartbeat = new HashMap<>();
    s_lastGoodPose = new HashMap<>();
    s_lastGoodPoseTime = new HashMap<>();

    // init heartbeat to avoid crash on enable
    s_limelightHeartbeat.put(Constants.Drive.SHOOTER_LIMELIGHT_NAME, 0.0);
    s_limelightHeartbeat.put(Constants.Drive.RIGHT_LIMELIGHT_NAME, 0.0);

    Logger.recordOutput("DriveSubsystem/percieved_alliance", DriverStation.getAlliance().toString());

    s_drivetrain = TunerConstants.createDrivetrain();

    // Operator-perspective drive request for teleop (driver sticks)
    s_drive =
      new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Drive.MAX_SPEED.times(Constants.Drive.DEADBAND_SCALAR))
        .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    s_autoAimDrive =
      new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Drive.MAX_SPEED.times(Constants.Drive.DEADBAND_SCALAR))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        
    s_autoDrive =
      new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    
    s_headingController = new PIDController(7, 0.0, 0.0);
    s_headingController.setTolerance(Degrees.of(1).in(Radians));
    s_headingController.enableContinuousInput(-Math.PI, Math.PI);

    s_autoAimController = new PIDController(6.5, 0.0, 0.0);
    // TODO adjust how much it divides by
    s_autoAimController.setTolerance(Degrees.of(1).in(Radians));
    s_autoAimController.enableContinuousInput(-Math.PI, Math.PI);

    s_autoDriveController = new PIDController(1.65, 0.0, 0.0);
    s_autoDriveController.setTolerance(0.04); // 4 cm tolerance

    // set throttle to idle here
    LimelightHelpers.SetThrottle(
      Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
    );
    LimelightHelpers.SetThrottle(
      Constants.Drive.RIGHT_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
    );

    //init limelight thread
    m_limelight_thread = new Thread(this::limelight_thread_func);
    m_limelight_thread.setDaemon(true);
    m_limelight_thread.start();
    
    Logger.recordOutput("DriveSubsystem/Odometry/target", new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  /**
   * Gets the latest MegaTag 1 pose from a given limelight and does filtering.
   * The things it filters based off of are:
   * <ul>
   *  <li> If no new pose estimate has been made by the limelight since
   * this method was last called.
   *  <li> If the original pose estimate object is null.
   *  <li> If the rotational rate of the drivetrain is greater than 2pi/second.
   *  <li> If there are no tags seen.
   *  <li> If any of the elements of the pose are NaN.
   *  <li> If the estimated pose is either outside the field, too high off the ground,
   * or too far below the ground.
   *  <li> If only seeing a single tag, reject estimates that are either too high in
   * ambiguity or too far from the tag.
   * </ul>
   * @param limelight Name of the limelight to get pose from.
   * @return Null if pose is rejected, PoseEstimate object if valid
   */
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

    if (Double.isNaN(pose_estimate.pose.getX()) || Double.isNaN(pose_estimate.pose.getY()) || Double.isNaN(pose_estimate.pose.toPose2d().getRotation().getDegrees())) {
      return null;
    }

    // filtering for unreasonable poses
    // robot isn't going to be outside the field
    // bump is 16 cm off the ground
    // and anything above 25cm is probably insane airtime & unreliable
    if (
      pose_estimate.pose.getX() < 0                                   ||
      pose_estimate.pose.getX() > Constants.Field.FIELD_X.in(Meters)  ||
      pose_estimate.pose.getY() < 0                                   ||
      pose_estimate.pose.getY() > Constants.Field.FIELD_Y.in(Meters)  ||
      pose_estimate.pose.getZ() < -0.05                               ||
      pose_estimate.pose.getZ() > 0.25
    ) {
      return null;
    }

    // aggressive filtering for one tag
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
    if (pose_estimate.tagCount == 1 && pose_estimate.rawFiducials.length == 1) {
      RawFiducial tag = pose_estimate.rawFiducials[0];
      // ignore anything that has too high ambiguity
      if (tag.ambiguity > Constants.Drive.SINGLE_TAG_AMBIGUITY_CUTOFF) {
        return null;
      }
      // we outright reject anything further than a certain distance
      if (tag.distToCamera > Constants.Drive.SINGLE_TAG_DISTANCE_CUTOFF) {
        return null;
      }
    }

    return pose_estimate;
  }

  /**
   * Handles all limelight pose estimation and adding vision measurements
   * to the drivetrain (while s_shouldDoGlobalPoseEstimation is true). Gets
   * the filtered pose estimate from each limelight, and ignores rejected
   * poses. If disabled, set standard deviations to 0.25 for all axes (x, y,
   * and rotation). If not disabled and seeing more than one tag, set standard
   * deviations to 0.5 in x and y and ignore rotation. If not disabled and only
   * seeing one tag, start at 0.5 and increase standard deviations when further
   * from the tag.
   */
  public void limelight_thread_func() {

    String[] limelights = {
      Constants.Drive.SHOOTER_LIMELIGHT_NAME,
      Constants.Drive.RIGHT_LIMELIGHT_NAME
    };

    while (true) {
      for (String limelight : limelights) {
        if (!s_shouldDoGlobalPoseEstimation) {
          continue;
        }

        LimelightHelpers.PoseEstimate pose_estimate =
          getFilteredLimelightPose(limelight);

        if (pose_estimate == null) {
          continue;
        }

        if (getState() == DriveStates.DISABLED) {
          // trust limelights for rotation if disabled
          s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.25, 0.25, 0.25));
        } else {
          // when running, ignore limelight rotation entirely
          if (pose_estimate.tagCount > 1) {
            // more than one tag
            s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
          } else {
            // if only one tag, increase uncertainty based on distance
            // starts at 0.5 and then increases with a factor of 0.3 for every
            // meter of distance
            double distanceStdDev =
              0.5 + (
                pose_estimate.rawFiducials[0].distToCamera *
                Constants.Drive.TAG_UNCERTAINTY_SCALING_FACTOR
              );
            s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(
              distanceStdDev,
              distanceStdDev,
              9999999
            ));
          }
        }

        s_drivetrain.addVisionMeasurement(
          pose_estimate.pose.toPose2d(), Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds)
        );

        s_lastGoodPose.put(
          limelight,
          pose_estimate.pose.toPose2d()
        );
        s_lastGoodPoseTime.put(
          limelight,
          Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds)
        );
      }
      try {
        Thread.sleep(15);
      } catch (InterruptedException e) {}
    }
  }

  /**
   * Set the fiducial filtering on every limelight to every tag. Basically,
   * tell them to not ignore any tags.
   */
  private void setAllLimelightsToAllTags() {
    LimelightHelpers.SetFiducialIDFiltersOverride(
      Constants.Drive.SHOOTER_LIMELIGHT_NAME,
      Constants.Drive.ALL_APRIL_TAGS
    );
    LimelightHelpers.SetFiducialIDFiltersOverride(
      Constants.Drive.RIGHT_LIMELIGHT_NAME,
      Constants.Drive.ALL_APRIL_TAGS
    );
  }

  /**
   * All of these should be relative to the driver.
   * @param driveRequest Front and back
   * @param strafeRequest Left and right
   * @param rotateRequest Rotation
   */
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
    Logger.recordOutput("DriveSubsystem/Odometry/lastGotoTimestamp", Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()));

    Pose2d robotPose = s_drivetrain.getState().Pose;
    Translation2d newPosition = target.getTranslation().minus(robotPose.getTranslation());
    double distance = robotPose.getTranslation().getDistance(target.getTranslation());

    Logger.recordOutput("DriveSubsystem/Odometry/radiansToRotate",
      Math.abs(
        MathUtil.angleModulus(
          robotPose.getRotation().getRadians() - target.getRotation().getRadians()
        )
      )
    );
    Logger.recordOutput("DriveSubsystem/Odometry/distance", distance);

    Rotation2d directionOfTravel = newPosition.getAngle();

    Logger.recordOutput("DriveSubsystem/Odometry/directionOfTravel", directionOfTravel);

    // The PID is trying to make distance 0, so its output is negative.
    // We negate it to get a positive "drive toward target" speed.
    double rawDriveOutput = -s_autoDriveController.calculate(distance, 0.0);

    // Force the robot to move at 0.2 m/s minimum unless it's closer than
    // GOTO_SETTLE_DISTANCE to the target
    double velocityFloor = distance > Constants.Drive.GOTO_SETTLE_DISTANCE.in(Meters) ? 0.2 : 0.0;

    double outputVelocity = 
      MathUtil.clamp(
        Math.max(
        rawDriveOutput,
        exitVelocity.in(MetersPerSecond)
      ) + velocityFloor,
      0,
      maxVelocity.in(MetersPerSecond)
    );

    double maxRotRad = maxRotationRate.in(RadiansPerSecond);
    double rotationRate = MathUtil.clamp(
      s_headingController.calculate(robotPose.getRotation().getRadians(), target.getRotation().getRadians()),
      -maxRotRad,
      maxRotRad
    );

    double xComponent = outputVelocity * directionOfTravel.getCos();
    double yComponent = outputVelocity * directionOfTravel.getSin();

    Logger.recordOutput("DriveSubsystem/Odometry/outputVelocity", outputVelocity);
    Logger.recordOutput("DriveSubsystem/Odometry/rotationRate", rotationRate);

    s_drivetrain.setControl(
      s_autoDrive
        .withVelocityX(MetersPerSecond.of(xComponent))
        .withVelocityY(MetersPerSecond.of(yComponent))
        .withRotationalRate(rotationRate)
    );
  }

  /**
   * Set the control of the drivetrain to not move at all. Do note that
   * it will continue to move if something else is telling it to (this is
   * not a method to stop all operation of the drivetrain).
   */
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

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      Logger.recordOutput("DriveSubsystem/settingOperatorPerspective", true);
      if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(Alliance.Red)) {
        Logger.recordOutput("DriveSubsystem/setOperatorPerspective", Alliance.Red);
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
      } else {
        Logger.recordOutput("DriveSubsystem/setOperatorPerspective", Alliance.Blue);
        s_drivetrain.setOperatorPerspectiveForward(
            CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
      }
      m_hasAppliedOperatorPerspective = true;
    } else {
      Logger.recordOutput("DriveSubsystem/settingOperatorPerspective", false);
    }

    s_lastGoodPose.keySet().forEach(
      (String key) -> {
        Logger.recordOutput(
          "DriveSubsystem/lastGoodLimelightPose/" + key,
          s_lastGoodPose.get(key)
        );
      }
    );
    s_lastGoodPoseTime.keySet().forEach(
      (String key) -> {
        Logger.recordOutput(
          "DriveSubsystem/lastGoodLimelightPoseTime/" + key,
          s_lastGoodPoseTime.get(key)
        );
      }
    );

    Logger.recordOutput("DriveSubsystem/doingGlobalPoseEstimation", s_shouldDoGlobalPoseEstimation);
    Logger.recordOutput("DriveSubsystem/percieved_alliance", DriverStation.getAlliance().toString());
    Logger.recordOutput("DriveSubsystem/resettingOdometry", resettingOdom);
    Logger.recordOutput("DriveSubsystem/inAllianceZone", inAllianceZone());
    Logger.recordOutput("DriveSubsystem/atShootingRotation", atShootingRotation());
    Logger.recordOutput("DriveSubsystem/subsystemState", getState().toString());
  }

  /**
   * Set the drive subsystem to be under "driver" control. If
   * in autonomous, this goes to the AUTO drive state. If in
   * teleop, this goes to the DRIVER_CONTROL drive state.
   */
  public void driverControl() {
    if (DriverStation.isAutonomous()) {
      s_requestedDriveState = DriveStates.AUTO;
    } else {
      s_requestedDriveState = DriveStates.DRIVER_CONTROL;
    }
  }

  /**
   * Set the drive subsystem state to auto aim.
   */
  public void driveAutoAim() {
    s_requestedDriveState = DriveStates.AUTO_AIM;
  }

  // // TODO nuke
  // public void driveOverRamp() {
  //   s_requestedDriveState = DriveStates.OVER_RAMP;
  // }

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
   * Check to see if robot is at a destination
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

  /**
   * Checks if the drivetrain is at the rotation desired by AimUtil.
   * The tolerance is defined by {@link frc.robot.Constants.Drive#ROTATION_TOLERANCE
   * ROTATION_TOLERANCE}.
   * @return True if within tolerance of target.
   */
  public boolean atShootingRotation() {
    double delta = Rotation2d.fromDegrees(AimUtil.getRobotHeading().in(Degrees)).minus(
      s_drivetrain.getState().Pose.getRotation()
    ).getMeasure().abs(Degrees);
    return delta <=
      Constants.Drive.ROTATION_TOLERANCE.in(Degrees);
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

  @Override
  public void close() throws Exception {
    s_drivetrain.close();
  }
}