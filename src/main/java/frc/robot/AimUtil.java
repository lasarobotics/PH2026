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

  /**
   * 
   * @param distance Linear distance from hub
   * @return Value of X velocity of the ball when shot stationary
   */
  public static double getVelocityXStationary(double distance) {
    double y_max = Constants.Field.MAX_BALL_Y_POS;
    double y_end = Constants.Field.END_BALL_Y_POS;
    double g = Constants.Field.GRAVITY_VALUE;

    double x_vel = distance * (Math.sqrt(g))/((Math.sqrt(2 * y_max)) + Math.sqrt(y_max - y_end));
    return x_vel;
  }

  /**
   * @return Value of Y velocity of the ball when shot stationary
   */
  public static double getVelocityYStationary() {
    double y_max = Constants.Field.MAX_BALL_Y_POS;
    double g = Constants.Field.GRAVITY_VALUE;

    double y_vel = Math.sqrt(y_max * 2 * g);
    return y_vel;
  }

  /**
   * @return Speed of the ball needed to shoot while stationary
   * from the current position
   */
  public static double getVelocitySpeedStationary() {
    SwerveDriveState driveState = DriveSubsystem.getDrivetrain().getState();
    Pose2d robotPose = driveState.Pose;

    return getVelocityFromPose(robotPose);
  }

  /**
   * @param robotPose Your current robot pose
   * @return Speed of the ball needed to shoot while stationary
   */
  public static double getVelocityFromPose(Pose2d robotPose) {
    Translation2d goalLocation = Constants.Field.HUB_COORDINATES;
    Translation2d targetVec = goalLocation.minus(robotPose.getTranslation());
    double dist = targetVec.getNorm();
    
    double x_vel = getVelocityXStationary(dist);
    double y_vel = getVelocityYStationary();
    
    return Math.sqrt(Math.pow(x_vel, 2) + Math.pow(y_vel, 2));
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

    ShooterMathResults results = runShooterMath(
      currentRobotSpeeds,
      currentRobotPose,
      currentAngularVelocity
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
   * @return A ShooterMathResults record with the wanted exit angle,
   * ball velocity, and robot heading
   */
  public static ShooterMathResults runShooterMath(
    ChassisSpeeds currentRobotSpeeds,
    Pose2d currentRobotPose,
    AngularVelocity currentAngularVelocity
  ) {

    double latency = Constants.Drive.ROBOT_LATENCY;
    double currentRobotHeading = currentRobotPose.getRotation().getRadians();

    // Project your movement forward 
    Translation2d futurePos = currentRobotPose.getTranslation().plus(
      new Translation2d(
        currentRobotSpeeds.vxMetersPerSecond, 
        currentRobotSpeeds.vyMetersPerSecond
      )
      .times(latency + Constants.Field.HANG_TIME)
      .plus(
        new Translation2d(
          currentAngularVelocity.magnitude() * Constants.Shooter.SHOOTER_OFFSET * Math.cos(currentRobotHeading),
          currentAngularVelocity.magnitude() * Constants.Shooter.SHOOTER_OFFSET * Math.sin(currentRobotHeading)
        )
      )
    );
    
    // TODO (important) account for passing?
    // Get your distance to the hub (using the future position)
    Translation2d goalLocation = Constants.Field.HUB_COORDINATES;
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();

    // Get your stationary shooter velocity 2d vector
    Translation2d shooterVelocityVec = new Translation2d(
      getVelocityXStationary(dist),
      getVelocityYStationary());
    
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

  /**
   * @return Returns the speed of the flywheel needed to shoot
   */
  public static double getFlyWheelRadiansPerSecond() {
    return (2 * ballVelocity.in(MetersPerSecond))/Constants.Shooter.FLYWHEEL_RADIUS;
  }

}

