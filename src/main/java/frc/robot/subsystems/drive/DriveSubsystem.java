package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.AimUtil;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

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
        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        if (s_requestedDriveState == DriveStates.CLIMB_ALIGN && s_driveSubsystem.inAllianceZone()) return CLIMB_ALIGN;
        
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
      public SystemState nextState() {
        if (s_requestedDriveState == DriveStates.DRIVER_CONTROL) return DRIVER_CONTROL;
        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        if (s_requestedDriveState == DriveStates.CLIMB_ALIGN && s_driveSubsystem.inAllianceZone()) return CLIMB_ALIGN;

        return this;
      }
    },
    CLIMB_ALIGN {
      @Override
      public void initialize() {
        s_climbAlignSpeed = Constants.Drive.MAX_SPEED.magnitude();
        s_climbRotationSpeed = Constants.Drive.MAX_ANGULAR_RATE.magnitude();
        s_climbAlignDistanceError = 0.2;
        s_climbAlignRotationError = 0.1;
      }
      @Override
      public void execute() {
        s_driveSubsystem.goTo(s_climbPosition, 0, DriveSubsystem.s_climbAlignSpeed, DriveSubsystem.s_climbRotationSpeed);
      }

      @Override
      public SystemState nextState() {
        if (s_driveSubsystem.atDestination(s_climbPosition, DriveSubsystem.s_climbAlignDistanceError, DriveSubsystem.s_climbAlignRotationError)) {
          DriveSubsystem.s_climbAlignSpeed = Constants.Drive.MAX_SPEED.magnitude()/4;
          DriveSubsystem.s_climbRotationSpeed = Constants.Drive.MAX_SPEED.magnitude()/2;
          DriveSubsystem.s_climbAlignDistanceError = 0.01;
          DriveSubsystem.s_climbAlignRotationError = 0.01;

          if (!DriverStation.isAutonomous() && s_driveSubsystem.atDestination(s_climbPosition, DriveSubsystem.s_climbAlignDistanceError, DriveSubsystem.s_climbAlignRotationError)) {
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
      public DriveStates nextState() {
        if (s_requestedDriveState == DriveStates.DRIVER_CONTROL) return DRIVER_CONTROL;
        if (s_requestedDriveState == DriveStates.AUTO_AIM) return AUTO_AIM;
        if (s_requestedDriveState == DriveStates.CLIMB_ALIGN && s_driveSubsystem.inAllianceZone()) return CLIMB_ALIGN;

        return this;
      }
    }
  }

  private static DriveSubsystem s_driveSubsystem;
  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private static final Pose2d[] redPoses = new Pose2d[]{Constants.Field.RED_TOWER};
  private static final Pose2d[] bluePoses = new Pose2d[]{Constants.Field.BLUE_TOWER};

  //TODO: add more when you get more WP destinations
  private static final int WP_CLIMB = 0;

  private static final Double DEADBAND_SCALAR = 0.085;

  private boolean m_hasAppliedOperatorPerspective = false;

  private static double s_climbAlignSpeed = Constants.Drive.MAX_SPEED.magnitude();
  private static double s_climbRotationSpeed = Constants.Drive.MAX_ANGULAR_RATE.magnitude();
  private static double s_climbAlignDistanceError = 0.2;
  private static double s_climbAlignRotationError = 0.1;

  private static ProfiledPIDController s_autoAimController;
  private static PIDController s_autoDrive;
  private static PIDController s_headingController;

  private static DriveStates s_requestedDriveState = DriveStates.NOTHING;

  private static Pose2d[] s_alliancePoses;

  public static Pose2d s_climbPosition;

  //camera vars
  protected final Thread m_limelight_thread;

  private static boolean s_shooterCameraSeesTag = false;
  private static boolean s_climberCameraSeesTag = false;
  private static boolean s_backCameraSeesTag = false;


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
    super(DriveStates.DRIVER_CONTROL);

    if (Robot.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      s_alliancePoses = redPoses;
    } else {
      s_alliancePoses = bluePoses;
    }

    Logger.recordOutput("DriveSubsystem/percieved_alliance", Robot.getAlliance().toString());


    s_drivetrain = TunerConstants.createDrivetrain();

    s_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);



    s_autoDrive = new PIDController(1.75, 0.0, 0.0);
    s_headingController = new PIDController(3, 0.0, 0.5);
    s_headingController.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Fix this with real values
    s_autoAimController = new ProfiledPIDController(0.0, 0.0, 0, null);

    // TODO: Initialize the climb position to left
    s_climbPosition = new Pose2d(); 

    //init limelight thread
    m_limelight_thread = new Thread(this::limelight_thread_func);
    m_limelight_thread.setDaemon(true);
    m_limelight_thread.start();
  }


   /**
   * Function to set up the LimeLights on the robot 
   */
  public void limelight_thread_func() {
    String[] limelights = {"limelight-shooter", "limelight-climber", "limelight-back"};

    while (true) {
      for (String limelight : limelights) {
        LimelightHelpers.SetIMUMode(limelight, DriverStation.isDisabled() ? 1 : 3);
        LimelightHelpers.setLimelightNTDouble(
            limelight, "throttle_set", DriverStation.isDisabled() ? 200 : 0);
        LimelightHelpers.SetRobotOrientation(
            limelight, s_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

         Logger.recordOutput(
             getName() + "/" + limelight + "/botpose",
             LimelightHelpers.getBotPose3d_wpiBlue(limelight));
        double[] poseEntry =
            LimelightHelpers.getLimelightNTDoubleArray(limelight, "botpose_orb_wpiblue");
         Logger.recordOutput(
             getName() + "/" + limelight + "/botpose_orb", LimelightHelpers.toPose3D(poseEntry));
        LimelightHelpers.PoseEstimate pose_estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

        if (pose_estimate == null) {
          if (limelight == "limelight-shooter") {
            s_shooterCameraSeesTag = false;
          }
          if (limelight == "limelight-climber") {
            s_climberCameraSeesTag = false;
          }
          if (limelight == "limelight-back") {
            s_backCameraSeesTag = false;
          }
          continue;
        }
        boolean doRejectUpdate = false;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
          int[] validIds = {6, 7, 8, 9, 10, 11};
          LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
        } else {
          int[] validIds = {17, 18, 19, 20, 21, 22};
          LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validIds);
        }
        if (s_drivetrain.getState().Speeds.omegaRadiansPerSecond > 2 * Math.PI) {
          doRejectUpdate = true;
        }
        // if (new Translation2d(s_drivetrain.getState().Speeds.vxMetersPerSecond, s_drivetrain.getState().Speeds.vyMetersPerSecond).getDistance(new Translation2d(0, 0)) > 2.0) {
        //   doRejectUpdate = true;
        // }

        if (pose_estimate.tagCount == 0) {
          doRejectUpdate = true;
        }

        if (Double.isNaN(pose_estimate.pose.getX()) || Double.isNaN(pose_estimate.pose.getY()) || Double.isNaN(pose_estimate.pose.getRotation().getDegrees())) {
          doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
          s_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
          s_drivetrain.addVisionMeasurement(
              pose_estimate.pose, Utils.fpgaToCurrentTime(pose_estimate.timestampSeconds));
          // Logger.recordOutput(getName() + "/" + limelight + "/botpose_orb", pose_estimate.pose);
        }
        if (limelight == "limelight-shooter") {
          s_shooterCameraSeesTag = !doRejectUpdate;
        }
        if (limelight == "limelight-climber") {
          s_climberCameraSeesTag = !doRejectUpdate;
				}
        if (limelight == "limelight-back") {
          s_backCameraSeesTag = !doRejectUpdate;
        }
      }
      try {
        Thread.sleep(15);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        // e.printStackTrace();
      }
    }
  }


  // TODO move all these bindings into headhoncho
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
  }

  /**
   * Auto-aligns to a specific target (in other words, goes to a specific target)
   * @param target The target that you want to go to
   * @param maxVelocity max velocity the robot can go
   * @param maxRotationRate max rate of rotation the robot can rotate at
   * @return whether ropbot has reached target or not
   */
  private void goTo(Pose2d target, double exitVelocity, double maxVelocity, double maxRotationRate) {
    Logger.recordOutput("DriveSubsystem/Odometry/target", target);

    Pose2d robotPose = s_drivetrain.getState().Pose;
    Translation2d newPosition = target.getTranslation().minus(robotPose.getTranslation());

    double distance = robotPose.getTranslation().getDistance(target.getTranslation());

    Logger.recordOutput("DriveSubsystem/Odometry/distance", distance);

    var directionOfTravel = newPosition.getAngle();

    Logger.recordOutput("DriveSubsystem/Odometry/directionOfTravel", directionOfTravel);


    var outputVelocity = 
        Math.min(Math.abs(s_autoDrive.calculate(distance, 0.0)) + 0.2 + exitVelocity, maxVelocity);

    var rotationRate = 
        Math.min(s_headingController.calculate(robotPose.getRotation().getRadians(), target.getRotation().getRadians()), maxRotationRate);

    var xComponent = outputVelocity * directionOfTravel.getCos();
    var yComponent = outputVelocity * directionOfTravel.getSin();

    s_drivetrain.setControl(
       s_drive
          .withVelocityX(MetersPerSecond.of(xComponent))
          .withVelocityY(MetersPerSecond.of(yComponent))
          .withRotationalRate(rotationRate)
        );
        Logger.recordOutput("DriveSubsystem/Odometry/radiansToRotate", Math.abs(robotPose.getRotation().getRadians() - target.getRotation().getRadians()));

    }

    @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      Logger.recordOutput(getName() + "/settingOperatorPerspective", true);
      if (Robot.getAlliance().orElse(DriverStation.Alliance.Blue) == Alliance.Red) {
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

    Logger.recordOutput(getName() + "/inAllianceZone", inAllianceZone());
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

  /**
   * Checks robot's alliance and then checks if robot is in its alliance zone
   * @return true if robot is in alliance zone, false oterwise
   */
  public boolean inAllianceZone() {
    if (Robot.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) && s_drivetrain.getState().Pose.getX() >= Constants.Field.RED_ZONE_X) {
        return true;
    } else if (Robot.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue) && s_drivetrain.getState().Pose.getX() <= Constants.Field.BLUE_ZONE_X) {
      return true;
    }
    return false;
  }

  /**
   * 
   * @param target position robot is trying to reach
   * @param acceptableDistanceError how much error is acceptable in terms of distance to the target
   * @param acceptableRotationError how much error is acceptable in terms of angle relative to desired Pose2d
   * @return true if robot is at destination, false otherwise
   */
  public boolean atDestination(Pose2d target, double acceptableDistanceError, double acceptableRotationError) {
    Pose2d robotPose = s_drivetrain.getState().Pose;
    double distance = robotPose.getTranslation().getDistance(target.getTranslation());
    
    if (Math.abs(distance) < acceptableDistanceError 
    && Math.abs(robotPose.getRotation().getRadians() - target.getRotation().getRadians()) < acceptableRotationError) {
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

  @Override
  public void close() throws Exception {
    s_drivetrain.close();
  }

}
