package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AimUtil {
    
  private static LinearVelocity ballVelocity = MetersPerSecond.of(0);
  private static Angle exitAngle = Degrees.of(45);
  private static Angle robotHeading = Radians.of(0);

  private static Translation2d targetPosition;
  private static double targetHeight;

  /**
   * 
   * @param distance Linear distance from target
   * @param targetHeight Final y position of ball
   * @return Value of X velocity of the ball when shot stationary
   */
  private static double getVelocityXStationary(
    double distance,
    double targetHeight
  ) {
    double y_max = Constants.Field.MAX_BALL_Y_POS;
    double y_end = targetHeight;
    double g = Constants.Field.GRAVITY_VALUE;

    double x_vel = distance * (Math.sqrt(g))/((Math.sqrt(2 * y_max)) + Math.sqrt(y_max - y_end));
    return x_vel;
  }

  /**
   * @return Value of Y velocity of the ball when shot stationary
   */
  private static double getVelocityYStationary() {
    double y_max = Constants.Field.MAX_BALL_Y_POS;
    double g = Constants.Field.GRAVITY_VALUE;

    double y_vel = Math.sqrt(y_max * 2 * g);
    return y_vel;
  }

  /**
   * Set the target position that the AimUtil method uses
   * for the periodic updateShooterConstants calculation.
   * @param targetPos x and y position of the target on the
   * field
   * @param targetH final height that the ball should be at
   */
  public static void setTarget(
    Translation2d targetPos,
    double targetH
  ) {
    if (
      targetPos.equals(targetPosition) &&
      targetH == targetHeight
    ) {
      return;
    }

    targetPosition = targetPos;
    targetHeight = targetH;

    AimUtil.updateShooterConstants();
  }

  /**
   * What this method does it calculates the ball speed, robot heading, and shooter angle needed to shoot
   * even while moving at an arbitary position on the field. It then sets the member variables of this class to those values
   * (you can then call the related get methods of this file to get stuff you need)
   */
  public static void updateShooterConstants() {
    SwerveDriveState driveState = DriveSubsystem.getDrivetrain().getState();
    ChassisSpeeds currentRobotSpeeds = driveState.Speeds;
    Pose2d currentRobotPose = driveState.Pose;
    AngularVelocity currentAngularVelocity = AngularVelocity.ofBaseUnits(driveState.Speeds.omegaRadiansPerSecond, RadiansPerSecond);

    if (targetPosition == null) {
      return;
    }

    ShooterMathResults results = runShooterMath(
      currentRobotSpeeds,
      currentRobotPose,
      currentAngularVelocity,
      targetPosition,
      targetHeight
    );

    ballVelocity = results.ballVelocity();
    exitAngle = results.exitAngle();
    robotHeading = results.robotHeading();
  }

  public record ShooterMathResults(
    Angle exitAngle,
    LinearVelocity ballVelocity,
    Angle robotHeading
  ){}

  /**
   * Given a chassis speeds, position, and angular velocity, calculate the
   * optimal shooting parameters to get a ball in the hub.
   * @param currentRobotSpeeds driveState.Speeds
   * @param currentRobotPose driveState.Pose
   * @param currentAngularVelocity driveState.Speeds.omegaRadiansPerSecond
   * (as an AngularVelocity object)
   * @param targetPos The position of the target
   * @param targetHeight The final height the ball should be at
   * @return A ShooterMathResults record with the wanted exit angle,
   * ball velocity, and robot heading
   */
  private static ShooterMathResults runShooterMath(
    ChassisSpeeds currentRobotSpeeds,
    Pose2d currentRobotPose,
    AngularVelocity currentAngularVelocity,
    Translation2d targetPos,
    double targetHeight
  ) {

    double latency = Constants.Drive.ROBOT_LATENCY;
    double currentRobotHeading = currentRobotPose.getRotation().getRadians();

    double hangTime = (
      (
        Math.sqrt(Constants.Field.MAX_BALL_Y_POS * 2) +
        Math.sqrt(Constants.Field.MAX_BALL_Y_POS -
                  targetHeight)
      ) / Math.sqrt(Constants.Field.GRAVITY_VALUE)
    );

    // Project your movement forward 
    Translation2d futurePos = currentRobotPose.getTranslation().plus(
      new Translation2d(
        currentRobotSpeeds.vxMetersPerSecond, 
        currentRobotSpeeds.vyMetersPerSecond
      )
      .times(latency + hangTime)
      .plus(
        new Translation2d(
          currentAngularVelocity.magnitude() * Constants.Shooter.SHOOTER_OFFSET * Math.cos(currentRobotHeading),
          currentAngularVelocity.magnitude() * Constants.Shooter.SHOOTER_OFFSET * Math.sin(currentRobotHeading)
        )
      )
    );
    
    // Get your distance to the target (using the future position)
    Translation2d targetVec = targetPos.minus(futurePos);
    double dist = targetVec.getNorm();

    // Get your stationary shooter velocity 2d vector
    Translation2d shooterVelocityVec = new Translation2d(
      getVelocityXStationary(dist, targetHeight),
      getVelocityYStationary()
    );
    
    return new ShooterMathResults(
      // the arctangent of the Y velocity and the X velocity is the shooterAngle
      Angle.ofBaseUnits(
        Math.atan2(
          shooterVelocityVec.getY(), shooterVelocityVec.getX()
        ),
        Radians
      ),
      
      // Pythagorean sum of the X velocity and the Z velocity is the shooter
      LinearVelocity.ofBaseUnits(
        Math.sqrt(
          Math.pow(shooterVelocityVec.getX(), 2) + Math.pow(shooterVelocityVec.getY(), 2)
        ),
        MetersPerSecond
      ),
      
      // The angle of the target vector is your robot heading
      targetVec.getAngle().getMeasure()
    );
  }

  /**
   * @return Returns the speed of the ball at any given point needed to shoot
   */
  public static LinearVelocity getBallVelocity() {
    return ballVelocity;
  }

  /**
   * @return Returns the exitAngle of the ball at any given point
   */
  public static Angle getExitAngle() {
    return exitAngle;
  }

  /**
   * @return Returns the robot heading needed to shoot at any given point
   */
  public static Angle getRobotHeading() {
    return robotHeading;
  }
}

