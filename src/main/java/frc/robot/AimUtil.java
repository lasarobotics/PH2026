package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AimUtil {
    
  private static LinearVelocity ballVelocity = MetersPerSecond.of(0);
  private static Angle exitAngle = Degrees.of(45);
  private static Angle lastRobotHeading = Radians.of(0);
  private static Angle robotHeading = Radians.of(0);
  private static Time hangTime = Seconds.of(1.5);

  private static Translation2d targetPosition;
  private static double targetHeight;

  /**
   * Get the X velocity of stationary ball shot
   * @param distance Linear distance from target
   * @param targetHeight Final y position of ball
   * @param maxBallYPos The Y value for the highest point of the ball's curve
   * @return Value of X velocity of the ball when shot stationary
   */
  private static double getVelocityXStationary(
    double distance,
    double targetHeight,
    double maxBallYPos
  ) {
    double y_max = maxBallYPos;
    double y_end = targetHeight;
    double g = Constants.Field.GRAVITY_VALUE;

    double x_vel = distance * (Math.sqrt(g))/(Math.sqrt(2 * y_max) + Math.sqrt(2*(y_max - y_end)));
    return x_vel;
  }

  /**
   * Get the Y velocity of stationary ball shot
   * @param maxBallYPos The Y value for the highest point of the ball's curve
   * @return Value of Y velocity of the ball when shot stationary
   */
  private static double getVelocityYStationary(double maxBallYPos) {
    double y_max = maxBallYPos;
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
    // Aimutil already updates periodically,
    // so we don't want to waste cycles recalculating
    // if it's the same position
    if (
      targetPos.equals(targetPosition) &&
      targetH == targetHeight
    ) {
      return;
    }

    targetPosition = targetPos;
    targetHeight = targetH;

    Logger.recordOutput("AimUtil/targetPosition", targetPosition);
    Logger.recordOutput("AimUtil/targetHeight", targetHeight);

    AimUtil.updateShooterConstants();
  }

  /**
   * Calculate the ball speed, robot heading, and shooter angle needed to shoot
   * even while moving at an arbitrary position on the field, and modify member variables to store these calculated values
   */
  public static void updateShooterConstants() {
    SwerveDriveState driveState = DriveSubsystem.getDrivetrain().getState();
    ChassisSpeeds currentRobotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
      driveState.Speeds,
      driveState.Pose.getRotation()
    );
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
      targetHeight,
      Constants.Field.MAX_BALL_Y_POS.getAsDouble()
    );

    ballVelocity = results.ballVelocity();
    exitAngle = results.exitAngle();
    lastRobotHeading = robotHeading;
    robotHeading = results.robotHeading();
    hangTime = results.hangTime();
  
    Logger.recordOutput("AimUtil/ballVelocity", ballVelocity.in(MetersPerSecond));
    double rotationsPerSecond =
      ballVelocity.in(MetersPerSecond) /
      (Math.PI * Constants.Shooter.SHOOTER_RADIUS.in(Meters));
    Logger.recordOutput("AimUtil/outputRevps",
      rotationsPerSecond
    );
    Logger.recordOutput("AimUtil/exitAngle", exitAngle.in(Degrees));
    Logger.recordOutput("AimUtil/lastRobotHeading", lastRobotHeading);
    Logger.recordOutput("AimUtil/robotHeading", robotHeading);
    Logger.recordOutput("AimUtil/hangTime", hangTime);
  }

  /**
   * Record to store calculated shooter data
   * @param exitAngle The exit angle of the ball
   * @param ballVelocity The ball's exit velocity
   * @param robotHeading Angle of robot[\]
   * @param hangTime Time the ball spends in the air
   */
  public record ShooterMathResults(
    Angle exitAngle,
    LinearVelocity ballVelocity,
    Angle robotHeading,
    Time hangTime
  ){}

  /**
   * Given a chassis speeds, position, and angular velocity, calculate the
   * optimal shooting parameters to get a ball in the hub.
   * @param currentRobotSpeeds driveState.Speeds
   * @param currentRobotPose driveState.Pose
   * @param currentAngularVelocity driveState.Speeds.omegaRadiansPerSecond
   * (as an AngularVelocity object)
   * @param targetPos The position of the target
   * @param targetHeight The final height the ball should be at, relative to the field
   * @param maxBallYPos The Y value for the highest point of the ball's curve,
   * relative to the field
   * @return A ShooterMathResults record with the wanted exit angle,
   * ball velocity, and robot heading
   */
  private static ShooterMathResults runShooterMath(
    ChassisSpeeds currentRobotSpeeds,
    Pose2d currentRobotPose,
    AngularVelocity currentAngularVelocity,
    Translation2d targetPos,
    double targetHeight,
    double maxBallYPos
  ) {
    targetHeight = targetHeight - Constants.Shooter.SHOOTER_OFFSET_Z.in(Meters);
    maxBallYPos = maxBallYPos - Constants.Shooter.SHOOTER_OFFSET_Z.in(Meters);

    double latency = Constants.Drive.ROBOT_LATENCY;
    double currentRobotHeading = currentRobotPose.getRotation().getRadians();

    double hangTime = (
      (
        Math.sqrt(maxBallYPos * 2) +
        Math.sqrt(
          2 * (
            maxBallYPos -
            targetHeight
          )
        )
      ) / Math.sqrt(Constants.Field.GRAVITY_VALUE)
    );

    // Project your movement forward
    Translation2d futurePos =
      currentRobotPose.getTranslation().plus(
        new Translation2d(
          Constants.Shooter.SHOOTER_OFFSET_X.in(Meters),
          Constants.Shooter.SHOOTER_OFFSET_Y.in(Meters)
        ).rotateBy(currentRobotPose.getRotation())
      ).plus(
        new Translation2d(
          currentRobotSpeeds.vxMetersPerSecond,
          currentRobotSpeeds.vyMetersPerSecond
        )
        // causes feedback
        // .plus(
        //   new Translation2d(
        //     currentAngularVelocity.magnitude() * Constants.Shooter.SHOOTER_DISTANCE_FROM_CENTER.in(Meters) * Math.sin(currentRobotHeading),
        //     currentAngularVelocity.magnitude() * Constants.Shooter.SHOOTER_DISTANCE_FROM_CENTER.in(Meters) * Math.cos(currentRobotHeading)
        //   )
        // )
        .times(latency + hangTime)
      );
    
    // Get your distance to the target (using the future position)
    Translation2d targetVec = targetPos.minus(futurePos);
    double dist = targetVec.getNorm();

    // Get your stationary shooter velocity 2d vector
    Translation2d shooterVelocityVec = new Translation2d(
      getVelocityXStationary(dist, targetHeight, maxBallYPos),
      getVelocityYStationary(maxBallYPos)
    );

    Logger.recordOutput("AimUtil/shooterDistToHub",
      futurePos.getDistance(targetPos)
    );
    Logger.recordOutput("AimUtil/futurePos",
      new Pose2d(futurePos, new Rotation2d(targetVec.getAngle().getMeasure()))
    );
    Angle rotationToLog = targetVec.getAngle().getMeasure().plus(Constants.Shooter.SHOOTER_ROTATION);
    if (rotationToLog.in(Radians) > Math.PI) {
      rotationToLog = rotationToLog.minus(Degrees.of(360));
    }
    Logger.recordOutput("AimUtil/wantedPos",
      new Pose2d(
        currentRobotPose.getX(),
        currentRobotPose.getY(),
        new Rotation2d(rotationToLog)
      )
    );
    Logger.recordOutput("AimUtil/targetVec", targetVec);
    Logger.recordOutput("AimUtil/shooterVelocityVec", shooterVelocityVec);
    
    return new ShooterMathResults(
      // the arctangent of the Y velocity and the X velocity is the shooterAngle
      Radians.of(
        Constants.Shooter.AIMUTIL_HOOD_ANGLE_SCALAR.get() *
        // Constants.Shooter.AIMUTIL_HOOD_ANGLE_SCALAR *
        Math.atan2(
          shooterVelocityVec.getY(), shooterVelocityVec.getX()
        )
      ).plus(
        Degrees.of(
          Constants.Shooter.AIMUTIL_HOOD_ANGLE_ADDEND.get()
          // Constants.Shooter.AIMUTIL_HOOD_ANGLE_ADDEND
        )
      ),

      // Pythagorean sum of the X velocity and the Z velocity is the shooter
      MetersPerSecond.of(
        Constants.Shooter.AIMUTIL_SHOOTER_SPEED_SCALAR.get() *
        // Constants.Shooter.AIMUTIL_SHOOTER_SPEED_SCALAR *
        Math.sqrt(
          Math.pow(shooterVelocityVec.getX(), 2) + Math.pow(shooterVelocityVec.getY(), 2)
        )
      ).plus(MetersPerSecond.of(
        // basically aimutil shooter speed addend should be in rev/s
        // so we convert to meters per second
        Constants.Shooter.AIMUTIL_SHOOTER_SPEED_ADDEND.get() *
        // Constants.Shooter.AIMUTIL_SHOOTER_SPEED_ADDEND *
        2 * Math.PI * Constants.Shooter.SHOOTER_RADIUS.in(Meters)
      )),

      // The angle of the target vector is your robot heading
      // plus 90 because shooter is on right side of robot
      targetVec.getAngle().getMeasure().plus(Constants.Shooter.SHOOTER_ROTATION),

      // hang time
      Seconds.of(hangTime)
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
   * @return Returns the robot heading calculated in the previous loop
   */
  public static Angle getLastRobotHeading() {
    return lastRobotHeading;
  }

  /**
   * @return Returns the robot heading needed to shoot at any given point
   */
  public static Angle getRobotHeading() {
    return robotHeading;
  }

  /**
   * @return Returns the time the ball will be in the air when leaving shooter
   */
  public static Time getHangTime() {
    return hangTime;
  }
}

