package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class Util {
    
  private double ballVelocity;
  private double exitAngle;
  private double robotHeading;

  public Util(DriveSubsystem driveSubsystem) {
    ballVelocity = 0.0;
    exitAngle = 0.0;
    robotHeading = 0.0;
  }

  /**
   * 
   * @param distance Linear distance from hub
   * @return Value of X velocity of the ball when shot stationary
   */
  public double getVelocityXStationary(double distance) {
    double y_max = Constants.Field.MAX_BALL_Y_POS;
    double y_end = Constants.Field.END_BALL_Y_POS;
    double g = Constants.Field.GRAVITY_VALUE;

    double x_vel = distance * (Math.sqrt(g))/((Math.sqrt(2 * y_max)) + Math.sqrt(y_max - y_end));
    return x_vel;
  }

  /**
   * @return Value of Y velocity of the ball when shot stationary
   */
  public double getVelocityYStationary() {
    double y_max = Constants.Field.MAX_BALL_Y_POS;
    double g = Constants.Field.GRAVITY_VALUE;

    double y_vel = Math.sqrt(y_max * 2 * g);
    return y_vel;
  }

  /**
   * @param robotPose Your current robot pose
   * @return Speed of the ball needed to shoot stationary
   */
  public double getVelocitySpeedStationary(Pose2d robotPose) {
    Translation2d goalLocation = Constants.Drive.HUB_COORDINATES;
    Translation2d targetVec = goalLocation.minus(robotPose.getTranslation());
    double dist = targetVec.getNorm();
    
    double x_vel = getVelocityXStationary(dist);
    double y_vel = getVelocityYStationary();
    
    return Math.sqrt(Math.pow(x_vel, 2) + Math.pow(y_vel, 2));

  } 

  /**
   * @param currentRobotSpeeds Your current robot speeds
   * @param currentRobotPose Your current robot pose
   * @param currentAngularVelocity Your current robot's angular velocity
   * 
   * What this method does it calculates the ball speed, robot heading, and shooter angle needed to shoot
   * even while moving at an arbitary position on the field. It then sets the member variables of this class to those values
   * (you can then call the related get methods of this file to get stuff you need)
   */
  public void setShooterConstants(ChassisSpeeds currentRobotSpeeds, 
              Pose2d currentRobotPose, 
              AngularVelocity currentAngularVelocity) {

    double latency = Constants.Drive.ROBOT_LATENCY;
    double currentRobotHeading = currentRobotPose.getRotation().getRadians();

    //Project your movement forward 
    Translation2d futurePos = currentRobotPose.getTranslation().plus(
          new Translation2d(
            currentRobotSpeeds.vxMetersPerSecond, 
            currentRobotSpeeds.vyMetersPerSecond).
            times(latency + Constants.Drive.HANG_TIME).plus(
          new Translation2d(
            currentAngularVelocity.magnitude() * Constants.Drive.SHOOTER_OFFSET * Math.cos(currentRobotHeading),
            currentAngularVelocity.magnitude() * Constants.Drive.SHOOTER_OFFSET * Math.sin(currentRobotHeading)
          )
         )
        );

    
    //Get your distance to the hub (using the future position)
    Translation2d goalLocation = Constants.Drive.HUB_COORDINATES;
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();


    //Get your stationary shooter velocity 2d vector
    Translation2d shooterVelocityVec = new Translation2d(
      getVelocityXStationary(dist),
      getVelocityYStationary());


    //the arctangent of the Y velocity and the X velocity is the shooterAngle
    double shooterAngle = Math.atan2(shooterVelocityVec.getY(), shooterVelocityVec.getX());

    //Pythagorean sum of the X velocity and the Z velocity is the shooter
    double shotVelocity = Math.sqrt(Math.pow(shooterVelocityVec.getX(), shooterVelocityVec.getY()));

    //The angle of the target vector is your robot heading
    double robotHeading = targetVec.getAngle().getRadians();


    this.exitAngle = shooterAngle;
    this.ballVelocity = shotVelocity;
    this.robotHeading = robotHeading;
  }

  /**
   * @return Returns the speed of the ball at any given point needed to shoot
   */
  public double getBallVelocity() {
    return ballVelocity;
  }

  /**
   * @return Returns the exitAngle of the ball at any given point
   */
  public double getExitAngle() {
    return exitAngle;
  }

  /**
   * @return Returns the robot heading needed to shoot at any given point
   */
  public double getRobotHeading() {
    return robotHeading;
  }

  /**
   * @return Returns the speed of the flywheel needed to shoot
   */
  public double getFlyWheelRadiansPerSecond() {
    return (2 * ballVelocity)/Constants.Drive.FLYWHEEL_RADIUS;
  }

}

