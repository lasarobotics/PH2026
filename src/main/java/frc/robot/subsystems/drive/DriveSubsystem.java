package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LoopTimer;
import frc.robot.generated.TunerConstants;

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
          Translation2d currentPoseTranslation = s_drivetrain.getState().Pose.getTranslation();
          double currentAngle = s_drivetrain.getState().Pose.getRotation().getRadians();
          Translation2d hubTranslation = Constants.Drive.HUB_COORDINATES;
          Translation2d angleTranslation = hubTranslation.minus(currentPoseTranslation);
          double angle = Math.atan2(angleTranslation.getY(), angleTranslation.getX());
          double output = s_autoAimController.calculate(currentAngle, angle);

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
                  output
                ));
          
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
        public SystemState nextState() {
          return this;
        }

      },
    }


  private static CommandSwerveDrivetrain s_drivetrain;
  private static SwerveRequest.FieldCentric s_drive;

  private static DoubleSupplier s_driveRequest = () -> 0;
  private static DoubleSupplier s_strafeRequest = () -> 0;
  private static DoubleSupplier s_rotateRequest = () -> 0;

  private static final Double DEADBAND_SCALAR = 0.085;

  private boolean m_hasAppliedOperatorPerspective = false;


  private static double s_driveSpeedScalar = Constants.Drive.FAST_SPEED_SCALAR;

  private static ProfiledPIDController s_autoAimController;
  private static ProfiledPIDController s_climbAutoAlignController;
  private static ProfiledPIDController s_autoIntakeController;

  public DriveSubsystem(Hardware driveHardware) {
    super(DriveStates.DRIVER_CONTROL);

    s_drivetrain = TunerConstants.createDrivetrain();

    s_drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    s_autoIntakeController = new ProfiledPIDController(
      1.0, 
      1.0, 
      1.0, 
      Constants.Drive.TURN_CONSTRAINTS);
  }

  /*
   * Binds the controls needed to drive for controller usage
   */
  public void bindControls(
      DoubleSupplier driveRequest, DoubleSupplier strafeRequest, DoubleSupplier rotateRequest) {
    s_driveRequest = driveRequest;
    s_strafeRequest = strafeRequest;
    s_rotateRequest = rotateRequest;
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware driveHardware = new Hardware();
    return driveHardware;
  }

  @Override
  public void periodic() {

    LoopTimer.addTimestamp(getName() + " Start");

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

}
