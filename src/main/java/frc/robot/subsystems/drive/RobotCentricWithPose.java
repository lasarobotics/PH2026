package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class RobotCentricWithPose implements SwerveRequest {
    // TargetX/TargetY are desired field positions even though the request drives robot-centric.
    public double TargetX;
    public double TargetY;

    public double TargetXFeedForward = 0;
    public double TargetYFeedForward = 0;

    public Rotation2d TargetDirection = new Rotation2d();
    public double TargetRateFeedforward = 0;

    /**
     * The allowable deadband of the request, in m/s.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request, in radians per second.
     */
    public double RotationalDeadband = 0;
    /**
     * The center of rotation the robot should rotate around.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * Heading controller used to maintain robot-centric orientation.
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    public PhoenixPIDController XController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController YController = new PhoenixPIDController(0, 0, 0);

    private final SwerveRequest.RobotCentric m_robotCentric = new SwerveRequest.RobotCentric();
    
    public RobotCentricWithPose() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        double toApplyOmega = TargetRateFeedforward +
            HeadingController.calculate(
                parameters.currentPose.getRotation().getRadians(),
                TargetDirection.getRadians(),
                parameters.timestamp
            );

        Translation2d translationError = new Translation2d(
            TargetX - parameters.currentPose.getX(),
            TargetY - parameters.currentPose.getY());
        // Convert field error into robot frame so requested motion stays robot-centric.
        Translation2d robotError = translationError.rotateBy(parameters.currentPose.getRotation().unaryMinus());

        double toApplyX = TargetXFeedForward +
            XController.calculate(
                0,
                robotError.getX(),
                parameters.timestamp
            );

        double toApplyY = TargetYFeedForward +
            YController.calculate(
                0,
                robotError.getY(),
                parameters.timestamp
            );

        return m_robotCentric
            .withVelocityX(toApplyX)
            .withVelocityY(toApplyY)
            .withRotationalRate(toApplyOmega)
            .withDeadband(Deadband)
            .withRotationalDeadband(RotationalDeadband)
            .withCenterOfRotation(CenterOfRotation)
            .withDriveRequestType(DriveRequestType)
            .withSteerRequestType(SteerRequestType)
            .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
            .apply(parameters, modulesToApply);
    }

    public RobotCentricWithPose withTargetX(double x) {
        this.TargetX = x;
        return this;
    }

    public RobotCentricWithPose withTargetX(Distance x) {
        this.TargetX = x.in(Meters);
        return this;
    }

    public RobotCentricWithPose withTargetY(double y) {
        this.TargetY = y;
        return this;
    }

    public RobotCentricWithPose withTargetY(Distance y) {
        this.TargetY = y.in(Meters);
        return this;
    }

    public RobotCentricWithPose withTargetPose(Pose2d pose) {
        this.TargetX = pose.getMeasureX().in(Meters);
        this.TargetY = pose.getMeasureY().in(Meters);
        this.TargetDirection = pose.getRotation();
        return this;
    }

    public RobotCentricWithPose withFeedforwardX(double newVelocityX) {
        this.TargetXFeedForward = newVelocityX;
        return this;
    }

    public RobotCentricWithPose withFeedforwardX(LinearVelocity newVelocityX) {
        this.TargetXFeedForward = newVelocityX.in(MetersPerSecond);
        return this;
    }

    public RobotCentricWithPose withFeedforwardY(double newVelocityY) {
        this.TargetYFeedForward = newVelocityY;
        return this;
    }

    public RobotCentricWithPose withFeedforwardY(LinearVelocity newVelocityY) {
        this.TargetYFeedForward = newVelocityY.in(MetersPerSecond);
        return this;
    }

    public RobotCentricWithPose withVelocityX(double newVelocityX) {
        this.TargetXFeedForward = newVelocityX;
        return this;
    }

    public RobotCentricWithPose withVelocityX(LinearVelocity newVelocityX) {
        this.TargetXFeedForward = newVelocityX.in(MetersPerSecond);
        return this;
    }

    public RobotCentricWithPose withVelocityY(double newVelocityY) {
        this.TargetYFeedForward = newVelocityY;
        return this;
    }

    public RobotCentricWithPose withVelocityY(LinearVelocity newVelocityY) {
        this.TargetYFeedForward = newVelocityY.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the TargetDirection parameter and returns itself.
     *
     * @param newTargetDirection Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withTargetDirection(Rotation2d newTargetDirection) {
        this.TargetDirection = newTargetDirection;
        return this;
    }

    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withTargetRateFeedforward(double newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward;
        return this;
    }
    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withTargetRateFeedforward(AngularVelocity newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward.in(RadiansPerSecond);
        return this;
    }

    public RobotCentricWithPose withRotationalRate(double newRotationalRate) {
        this.TargetRateFeedforward = newRotationalRate;
        return this;
    }

    public RobotCentricWithPose withRotationalRate(AngularVelocity newRotationalRate) {
        this.TargetRateFeedforward = newRotationalRate.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withDeadband(double newDeadband) {
        this.Deadband = newDeadband;
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withDeadband(LinearVelocity newDeadband) {
        this.Deadband = newDeadband.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withRotationalDeadband(double newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband;
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withRotationalDeadband(AngularVelocity newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withCenterOfRotation(Translation2d newCenterOfRotation) {
        this.CenterOfRotation = newCenterOfRotation;
        return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public RobotCentricWithPose withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }
}
