package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

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
        s_shouldDoGlobalPoseEstimation = false;
      }

      @Override
      public void execute() {
        // turn on shooter at a throttle
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
        );

        s_driveSubsystem.setAllLimelightsToAllTags();

        LimelightHelpers.PoseEstimate pose_estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Drive.SHOOTER_LIMELIGHT_NAME);

        s_drivetrain.resetPose(pose_estimate.pose);
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
        // turn on all at full
        LimelightHelpers.SetThrottle(
          Constants.Drive.SHOOTER_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        LimelightHelpers.SetThrottle(
          Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = true;
        
        s_driveSubsystem.setAllLimelightsToAllTags();
      }

      @Override 
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return DRIVER_CONTROL;
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
        LimelightHelpers.SetThrottle(
          Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
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
          LimelightHelpers.SetThrottle(
            Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_IDLE
          );
          s_shouldDoGlobalPoseEstimation = true;

          s_driveSubsystem.setAllLimelightsToAllTags();
        } else {
          // in alliance zone, need accurate position
          // ergo local
          LimelightHelpers.SetThrottle(
            Constants.Drive.CLIMB_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
          );
          LimelightHelpers.SetThrottle(
            Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
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
          LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Drive.SHOOTER_LIMELIGHT_NAME);

        s_drivetrain.resetPoseNotGyro(pose_estimate.pose);

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
        LimelightHelpers.SetThrottle(
          Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
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
          LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Drive.SHOOTER_LIMELIGHT_NAME);

        s_drivetrain.resetPoseNotGyro(pose_estimate.pose);

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
            !DriverStation.isAutonomous() &&
            atDestination(
              s_climbPosition,
              DriveSubsystem.s_climbAlignDistanceError,
              DriveSubsystem.s_climbAlignRotationError
            )
          ) {
            return SLOW_DRIVER_ALIGN;
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
        LimelightHelpers.SetThrottle(
          Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_OFF
        );
        s_shouldDoGlobalPoseEstimation = true;

        s_driveSubsystem.setAllLimelightsToAllTags();
      }

      @Override
      public void execute() {
        LimelightHelpers.PoseEstimate pose_estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Drive.SHOOTER_LIMELIGHT_NAME);

        s_drivetrain.resetPoseNotGyro(pose_estimate.pose);

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

      @Override
      public void initialize() {
        sequenceIndex = 0;
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
        LimelightHelpers.SetThrottle(
          Constants.Drive.BACK_LIMELIGHT_NAME, Constants.Drive.THROTTLE_RUNNING
        );
        s_shouldDoGlobalPoseEstimation = true;

        s_driveSubsystem.setAllLimelightsToAllTags();

        Logger.recordOutput("DriveSubsystem/currentRampSequence", rampSequence);
      }

      @Override
      public void execute() {
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
          // when all 3 ramp pos are done, js stop the robot at the last pos
          s_drivetrain.setControl(
            s_drive
              .withVelocityX(MetersPerSecond.of(0.0))
              .withVelocityY(MetersPerSecond.of(0.0))
              .withRotationalRate(0.0));
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
        // driver must hold ramp button throughout the ramp
        // if driver lets go of ramp button goes back to driver_control
        if (!HeadHoncho.getInstance().wantToCrossRamp() || HeadHoncho.getInstance().wantToCancel()) { // allow driver to go back to drive state if let go
          s_requestedDriveState = DriveStates.DRIVER_CONTROL;
          return DRIVER_CONTROL;
        }
        if (sequenceIndex > 2) { // once it reaches last "stage" of ramp it goes to driver
          s_requestedDriveState = DriveStates.DRIVER_CONTROL;
          return DRIVER_CONTROL;
        }

        return this;
      }
    }
  }

  private static DriveSubsystem s_driveSubsystem;
  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;

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

  private boolean m_hasAppliedOperatorPerspective = false;

  private static LinearVelocity s_climbAlignSpeed = Constants.Drive.MAX_SPEED;
  private static AngularVelocity s_climbRotationSpeed = Constants.Drive.MAX_ANGULAR_RATE;
  private static Distance s_climbAlignDistanceError = Meters.of(0.2);
  private static Angle s_climbAlignRotationError = Radians.of(0.1);

  private static PIDController s_headingController;
  private static PIDController s_autoAimController;
  private static PIDController s_autoDrive;

  private static DriveStates s_requestedDriveState;

  private static Pose2d[] s_alliancePoses;

  public static Pose2d s_climbPosition;

  //camera vars
  protected final Thread m_limelight_thread;

  private static Map<String, Boolean> s_limelightSeesTag;
  private static Map<String, Double> s_limelightHeartbeat;

  private static boolean s_shouldDoGlobalPoseEstimation = false;

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

    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      s_alliancePoses = redPoses;
    } else {
      s_alliancePoses = bluePoses;
    }

    Logger.recordOutput("DriveSubsystem/percieved_alliance", DriverStation.getAlliance().toString());

    s_drivetrain = TunerConstants.createDrivetrain();

    s_drive =
      new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
        .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    
    s_headingController = new PIDController(3, 0.0, 0.0);
    s_headingController.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Fix this with real values
    // also add tolerance
    s_autoAimController = new PIDController(5, 0.0, 0.0);
    s_autoAimController.enableContinuousInput(-Math.PI, Math.PI);

    s_autoDrive = new PIDController(1.75, 0.0, 0.0);

    // TODO: Initialize the climb position to left
    s_climbPosition = new Pose2d(); 

    //init limelight thread
    m_limelight_thread = new Thread(this::limelight_thread_func);
    m_limelight_thread.setDaemon(true);
    m_limelight_thread.start();
  }

  private Optional<LimelightHelpers.PoseEstimate> getFilteredLimelightPose(String limelight) {
    LimelightHelpers.PoseEstimate pose_estimate =
      LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

    double hb = LimelightHelpers.getHeartbeat(limelight);
    
    if (s_limelightHeartbeat.get(limelight) == hb) {
      return Optional.empty();
    }
    s_limelightHeartbeat.put(
      limelight,
      hb
    );

    if (pose_estimate == null) {
      return Optional.empty();
    }

    if (Math.abs(s_drivetrain.getState().Speeds.omegaRadiansPerSecond) > 2 * Math.PI) {
      return Optional.empty();
    }

    if (pose_estimate.tagCount == 0) {
      return Optional.empty();
    }

    if (Double.isNaN(pose_estimate.pose.getX()) || Double.isNaN(pose_estimate.pose.getY()) || Double.isNaN(pose_estimate.pose.getRotation().getDegrees())) {
      return Optional.empty();
    }

    return Optional.of(pose_estimate);
  }

  /**
   * Function to set up the LimeLights on the robot 
   */
  public void limelight_thread_func() {

    s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

    String[] limelights = {
      Constants.Drive.SHOOTER_LIMELIGHT_NAME,
      Constants.Drive.CLIMB_LIMELIGHT_NAME,
      Constants.Drive.BACK_LIMELIGHT_NAME
    };

    while (true) {
      for (String limelight : limelights) {
        if (!s_shouldDoGlobalPoseEstimation) {
          continue;
        }

        Optional<LimelightHelpers.PoseEstimate> pose_estimate_maybe =
          getFilteredLimelightPose(limelight);

        if (pose_estimate_maybe.isEmpty()) {
          s_limelightSeesTag.put(
            limelight,
            false
          );

          continue;
        }

        LimelightHelpers.PoseEstimate pose_estimate = pose_estimate_maybe.get();

        s_drivetrain.addVisionMeasurement(
          pose_estimate.pose, Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds)
        );
        Logger.recordOutput(getName() + "/" + limelight + "/pose_estimate", pose_estimate.pose);

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
    LimelightHelpers.SetFiducialIDFiltersOverride(
      Constants.Drive.BACK_LIMELIGHT_NAME,
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
   * Auto-aligns to a specific target (in other words, goes to a specific target)
   * @param target The target that you want to go to
   * @param exitVelocity The target speed of the robot once done
   * @param maxVelocity max velocity the robot can go
   * @param maxRotationRate max rate of rotation the robot can rotate at
   * @return whether ropbot has reached target or not
   */
  public void goTo(
    Pose2d target,
    LinearVelocity exitVelocity,
    LinearVelocity maxVelocity,
    AngularVelocity maxRotationRate
  ) {
    Logger.recordOutput("DriveSubsystem/Odometry/target", target);

    Pose2d robotPose = s_drivetrain.getState().Pose;
    Translation2d newPosition = target.getTranslation().minus(robotPose.getTranslation());

    double distance = robotPose.getTranslation().getDistance(target.getTranslation());

    Logger.recordOutput("DriveSubsystem/Odometry/distance", distance);

    var directionOfTravel = newPosition.getAngle();

    Logger.recordOutput("DriveSubsystem/Odometry/directionOfTravel", directionOfTravel);

    var outputVelocity = 
      Math.min(Math.abs(s_autoDrive.calculate(distance, 0.0)) + 0.2 + exitVelocity.in(MetersPerSecond), maxVelocity.in(MetersPerSecond));

    var rotationRate = 
      Math.min(s_headingController.calculate(robotPose.getRotation().getRadians(), target.getRotation().getRadians()), maxRotationRate.in(RadiansPerSecond));

    var xComponent = outputVelocity * directionOfTravel.getCos();
    var yComponent = outputVelocity * directionOfTravel.getSin();

    s_drivetrain.setControl(
      s_drive
        .withVelocityX(MetersPerSecond.of(xComponent).times(-1))
        .withVelocityY(MetersPerSecond.of(yComponent).times(-1))
        .withRotationalRate(rotationRate)
    );
  
    Logger.recordOutput("DriveSubsystem/Odometry/radiansToRotate", Math.abs(robotPose.getRotation().getRadians() - target.getRotation().getRadians()));
  }

  public void stopMoving() {
    s_drivetrain.setControl(
      s_drive
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
      Logger.recordOutput(getName() + "/settingOperatorPerspective", true);
      if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(Alliance.Red)) {
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
    s_requestedDriveState = DriveStates.DRIVER_CONTROL;
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
   * @return true if robot is in alliance zone, false oterwise
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
   * (in radians)
   * @return true if robot is at destination, false otherwise
   */
  public static boolean atDestination(
    Pose2d target,
    Distance acceptableDistanceError,
    Angle acceptableRotationError
  ) {
    Pose2d robotPose = s_drivetrain.getState().Pose;
    double distance = Math.abs(robotPose.getTranslation().getDistance(target.getTranslation()));
    
    if (
      distance < acceptableDistanceError.in(Meters)
      && robotPose.getRotation().getMeasure().isNear(
        target.getRotation().getMeasure(),
        acceptableRotationError
      )
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
    switch (selectedValue) {
      case "Right":
        // TODO: Make this set the position to the right climb pose
        s_climbPosition = new Pose2d();
        break;
      case "Center":
        // TODO: Make this set the position to the center climb pose
        s_climbPosition = new Pose2d();
        break;
      default:
        // TODO: Make this set the position to the left climb pose
        s_climbPosition = new Pose2d();
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