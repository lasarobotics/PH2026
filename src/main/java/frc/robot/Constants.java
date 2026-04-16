// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Auto {
    public static final Time EightBallShootingTime = Seconds.of(7);

    public static final Distance VERY_HIGH_DISTANCE_TOLERANCE = Meters.of(0.5);
    public static final Distance HIGH_DISTANCE_TOLERANCE = Meters.of(0.3);
    public static final Distance LOW_DISTANCE_TOLERANCE = Meters.of(0.05);

    public static final Angle ULTRA_HIGH_ROTATION_TOLERANCE = Degrees.of(45);
    public static final Angle VERY_HIGH_ROTATION_TOLERANCE = Degrees.of(20);
    public static final Angle HIGH_ROTATION_TOLERANCE = Degrees.of(10);
    public static final Angle LOW_ROTATION_TOLERANCE = Degrees.of(2.5); 

    public static Pose2d mirrorY(Pose2d poseToMirror) {
      return new Pose2d(
        new Translation2d(
          poseToMirror.getMeasureX(),
          Field.FIELD_Y.minus(poseToMirror.getMeasureY())
        ),
        poseToMirror.getRotation().times(-1)
      );
    }

    public static Pose2d flipHeading(Pose2d poseToFlip) {
      return new Pose2d(
        poseToFlip.getTranslation(),
        poseToFlip.getRotation().times(-1)
      );
    }

    public static Pose2d rotate180(Pose2d poseToFlip) {
      return new Pose2d(
        poseToFlip.getTranslation(),
        poseToFlip.getRotation().plus(Rotation2d.k180deg)
      );
    }

    //                                     //
    // standard poses inside alliance zone //
    //                                     //

    // move 1.5 meters over depot
    public static final Distance DEPOT_EXIT_OFFSET = Meters.of(1.5);

    public static final Pose2d BLUE_RIGHT_POSE = new Pose2d(
      new Translation2d(
        Meters.of(2.5),
        Meters.of(1.542)
      ),
      new Rotation2d(
        Degrees.of(-135)
      )
    );
    public static final Pose2d BLUE_LEFT_POSE = mirrorY(BLUE_RIGHT_POSE);
    public static final Pose2d BLUE_CENTER_POSE = new Pose2d(
      new Translation2d(
        BLUE_RIGHT_POSE.getMeasureX(),
        Field.FIELD_Y.div(2)
      ),
      Rotation2d.k180deg
    );
    public static final Pose2d BLUE_DEPOT_ENTER_POSE = new Pose2d(
      new Translation2d(
        Meters.of(0.5),
        Meters.of(7)
      ),
      Rotation2d.kCW_90deg
    );
    public static final Pose2d BLUE_DEPOT_EXIT_POSE = new Pose2d(
      new Translation2d(
        BLUE_DEPOT_ENTER_POSE.getMeasureX(),
        BLUE_DEPOT_ENTER_POSE.getMeasureY().minus(DEPOT_EXIT_OFFSET)
      ),
      BLUE_DEPOT_ENTER_POSE.getRotation()
    );

    //                                     //
    // bump traversal & neutral zone poses //
    //                                     //

    //              //
    //  blue right  //
    //              //

    // bump crossing
    public static final Pose2d BLUE_RIGHT_ABUMP_AZ_POSE = new Pose2d(
      new Translation2d(
        Meters.of(3.1),
        Meters.of(2.375)
      ),
      Rotation2d.k180deg
    );
    public static final Pose2d BLUE_RIGHT_ABUMP_NZ_POSE = new Pose2d(
      new Translation2d(
        Meters.of(5.983),
        BLUE_RIGHT_ABUMP_AZ_POSE.getMeasureY()
      ),
      Rotation2d.kZero
    );

    // first tap
    public static final Pose2d BLUE_RIGHT_DEPOT_NZ_POSE = new Pose2d(
      new Translation2d(
        Meters.of(7.5),
        Meters.of(6.16)
      ),
      new Rotation2d(Degrees.of(45))
    );
    public static final Pose2d BLUE_RIGHT_HUB_NZ_POSE = new Pose2d(
      new Translation2d(
        Meters.of(7.9),
        Meters.of(3.5)
      ),
      Rotation2d.kCCW_90deg
    );
    public static final Pose2d BLUE_RIGHT_OUTPOST_NZ_POSE = new Pose2d(
      new Translation2d(
        Meters.of(7.8),
        Meters.of(3)
      ),
      new Rotation2d(Degrees.of(45))
    );
    public static final Pose2d BLUE_RIGHT_PLOW_1_NZ_POSE = new Pose2d(
      new Translation2d(
        Meters.of(7.2),
        Meters.of(2.2)
      ), new Rotation2d(Degrees.of(0))
    );

    // double tap stuff
    public static final Pose2d BLUE_RIGHT_DEPOT_CLOSE_NZ_POSE = new Pose2d(
      new Translation2d(
        BLUE_RIGHT_DEPOT_NZ_POSE.getMeasureX().minus(Meters.of(1.5)),
        BLUE_RIGHT_DEPOT_NZ_POSE.getMeasureY().minus(Meters.of(1.5))
      ),
      BLUE_RIGHT_DEPOT_NZ_POSE.getRotation()
    );
    public static final Pose2d BLUE_RIGHT_HUB_CLOSE_NZ_POSE = new Pose2d(
      new Translation2d(
        BLUE_RIGHT_HUB_NZ_POSE.getMeasureX().minus(Meters.of(1.0)),
        BLUE_RIGHT_HUB_NZ_POSE.getMeasureY().plus(Meters.of(1.5))
      ),
      BLUE_RIGHT_HUB_NZ_POSE.getRotation()
    );
    public static final Pose2d BLUE_RIGHT_OUTPOST_CLOSE_NZ_POSE = new Pose2d(
      new Translation2d(
        BLUE_RIGHT_OUTPOST_NZ_POSE.getMeasureX().minus(Meters.of(1.5)),
        BLUE_RIGHT_OUTPOST_NZ_POSE.getMeasureY()
      ),
      Rotation2d.kCCW_90deg
    );
  
    //             //
    //  blue left  //
    //             //

    public static final Pose2d BLUE_LEFT_ABUMP_AZ_POSE = mirrorY(BLUE_RIGHT_ABUMP_AZ_POSE);
    public static final Pose2d BLUE_LEFT_ABUMP_NZ_POSE = mirrorY(BLUE_RIGHT_ABUMP_NZ_POSE);
    public static final Pose2d BLUE_LEFT_DEPOT_NZ_POSE = flipHeading(BLUE_RIGHT_DEPOT_NZ_POSE);
    public static final Pose2d BLUE_LEFT_HUB_NZ_POSE = mirrorY(BLUE_RIGHT_HUB_NZ_POSE);
    public static final Pose2d BLUE_LEFT_OUTPOST_NZ_POSE = new Pose2d(new Translation2d(Meters.of(7.8), Meters.of(5)), new Rotation2d(Degrees.of(-45)));
    public static final Pose2d BLUE_LEFT_DEPOT_CLOSE_NZ_POSE = flipHeading(BLUE_RIGHT_DEPOT_CLOSE_NZ_POSE);
    public static final Pose2d BLUE_LEFT_HUB_CLOSE_NZ_POSE = mirrorY(BLUE_RIGHT_HUB_CLOSE_NZ_POSE);
    public static final Pose2d BLUE_LEFT_OUTPOST_CLOSE_NZ_POSE = flipHeading(BLUE_RIGHT_OUTPOST_CLOSE_NZ_POSE);
    public static final Pose2d BLUE_LEFT_PLOW_1_NZ_POSE = mirrorY(BLUE_RIGHT_PLOW_1_NZ_POSE);
  }

  public static class Shooter {
    public static final int LEADER_SHOOTER_MOTOR_ID = 30;
    public static final int FOLLOWER_SHOOTER_ONE_MOTOR_ID = 31;
    public static final int FOLLOWER_SHOOTER_TWO_MOTOR_ID = 32;
    public static final int INDEXER_MOTOR_ID = 33;
    public static final int HOOD_MOTOR_ID = 34;
    public static final int HOOD_CANCODER_ID = 35;
    public static final int VERT_ROLLER_MOTOR_ID = 36;
    public static final int BELT_MOTOR_ID = 37;

    // duty cycle
    // should be negative
    public static final double INDEXER_MOTOR_SPEED = -1.0;
    // duty cycle
    public static final double VERT_ROLLER_MOTOR_SPEED = 0.5;
    public static final double BELT_MOTOR_SPEED = 0.85;
    public static final double REVERSE_BELT_MOTOR_SPEED = -0.75;
    // spin up voltage for flywheel
    // 3.9V settles at just about 40 rotations per second
    // generally I think we shoot at around 40 to 50 rotations per second
    // chris suggests going higher so we're above rather than below
    // call it 5 volts tbh
    public static final double HOLD_VOLTAGE = 5;

    // shooter speed tolerance is in rotations per second
    public static final double SHOOTER_SPEED_BELOW_TOLERANCE = 1;
    public static final double SHOOTER_SPEED_ABOVE_TOLERANCE = 6;
    public static final Angle HOOD_POSITION_TOLERANCE = Degrees.of(3.75);

    // how long we can be shooting after the end of the shift
    // this accounts for hang time:
    // basically we can shoot if time in shift - hangtime >= margin
    // so with 1.5s hangtime and 0s left in shift we have -1.5s
    // hub takes like 1 to 2 seconds to count & counts for 3 seconds after end of period
    // so -1.5 is probably alright
    public static final double SHOOTER_TIME_MARGIN = -1.5;

    // seconds
    public static final double SPIN_UP_TIME = 2;

    public static final Distance SHOOTER_RADIUS = Inches.of(2);
    public static final double FLYWHEEL_RADIUS = 0.0508;

    // Note: using wpilib coordinate system
    // intake is front
    // x offset from center (assume middle of shooter)
    // (behind center)
    public static final Distance SHOOTER_OFFSET_X = Meters.of(-0.126059); // measured in cad
    // y offset from center
    // (left of center)
    public static final Distance SHOOTER_OFFSET_Y = Meters.of(0.121617); // measured in cad
    // z offset from floor (exit angle 60deg)
    public static final Distance SHOOTER_OFFSET_Z = Meters.of(0.548303); // measured in cad

    public static final Angle SHOOTER_ROTATION = Degrees.of(180);

    public static final Distance SHOOTER_DISTANCE_FROM_CENTER = Meters.of(
      Math.sqrt(
        Math.pow(SHOOTER_OFFSET_X.in(Meters), 2) + Math.pow(SHOOTER_OFFSET_Y.in(Meters), 2)
      )
    );

    // these values are for when the robot is pressed againt the tower
    public static final Angle DUMB_HOOD_POSITION = Degrees.of(-18.906);
    // public static final LoggedNetworkNumber DUMB_HOOD_POSITION
    //   = new LoggedNetworkNumber("/Tuning/dumbHoodPosition", -18.906);
    // In rotations per second
    // currently based on empirical measurement
    public static final double DUMB_SHOOTER_SPEED = 42.148;
    // public static final LoggedNetworkNumber DUMB_SHOOTER_SPEED
    //   = new LoggedNetworkNumber("/Tuning/dumbShooterSpeed", 42.148);

    public static LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_SCALAR
      = new LoggedNetworkNumber("/Tuning/aimutilSpeedScalar", 1.05);
    public static LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_ADDEND
      = new LoggedNetworkNumber("/Tuning/aimutilSpeedFudger", 1.0);
    public static LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_SCALAR
      = new LoggedNetworkNumber("/Tuning/hoodAngleScalar", 1);
    public static LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_ADDEND
      = new LoggedNetworkNumber("/Tuning/hoodAngleFudger", -3);
    // public static final double AIMUTIL_SHOOTER_SPEED_SCALAR = 1;
    // public static final double AIMUTIL_SHOOTER_SPEED_ADDEND = -0.75;
    // public static final double AIMUTIL_HOOD_ANGLE_SCALAR = 1;
    // public static final double AIMUTIL_HOOD_ANGLE_ADDEND = -5;
  }

  public static class Drive {
    public static final Distance WHEEL_FROM_CENTER_DIST =
      Meters.of(
        Math.sqrt(
          Math.pow(Inches.of(9.375).in(Meters), 2) +
          Math.pow(Inches.of(12.375).in(Meters), 2)
        )
      );
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    // map linear speed onto circumference
    // approximately 2.066 rotations per second
    // or 12.981 radians
    public static final AngularVelocity MAX_ANGULAR_RATE =
      RotationsPerSecond.of(
        MAX_SPEED.in(MetersPerSecond) /
        (2 * Math.PI * WHEEL_FROM_CENTER_DIST.in(Meters))
      );

    // normally 0.3 and 1.0
    public static final double SLOW_SPEED_SCALAR = 0.15;
    public static final double FAST_SPEED_SCALAR = .75;
    public static final double ROTATION_SPEED_SCALAR = .75 * .85;

    // P on rotation error for auto aim
    public static final LoggedNetworkNumber AIM_ROTATION_P =
      new LoggedNetworkNumber("/Tuning/aimRotationP", 7);
    // D on rotation error for auto aim
    public static final LoggedNetworkNumber AIM_ROTATION_D =
      new LoggedNetworkNumber("/Tuning/aimRotationD", 0.5);

    // meters per second
    public static final double MAX_SHOOTING_SPEED = 2.0;

    public static final Double DEADBAND_SCALAR = 0.085;
    public static final Double AUTO_DEADBAND_SCALAR = 0.02;
    public static final Double AUTO_ROTATIONAL_DEADBAND_SCALAR = 0.02;

    // Distance threshold below which we remove the velocity floor
    // to allow the robot to actually settle at the target
    public static final Distance GOTO_SETTLE_DISTANCE = Meters.of(.15);

    public static final String SHOOTER_LIMELIGHT_NAME = "limelight-shooter";
    public static final String RIGHT_LIMELIGHT_NAME = "limelight-right";
    public static final double SINGLE_TAG_AMBIGUITY_CUTOFF = 0.5;
    public static final double SINGLE_TAG_DISTANCE_CUTOFF = 5;
    // 5 meters should be about 2 meters of stddev (feels about right)
    // the base stddev is 0.5 (defined in code in drivesubsystem)
    // 1.5 / 5 = 0.3
    public static final double TAG_UNCERTAINTY_SCALING_FACTOR = 0.3;

    public static final int THROTTLE_OFF = 200;
    public static final int THROTTLE_IDLE = 0;
    public static final int THROTTLE_RUNNING = 0;

    public static final int[] ALL_APRIL_TAGS = new int[]{};
    public static final int[] RED_TOWER_APRIL_TAGS = new int[]{
      15, 16
    };
    public static final int[] RED_HUB_APRIL_TAGS = new int[]{
      2, 3, 4, 5, 8, 9, 10, 11
    };
    public static final int[] RED_TOWER_AND_HUB_APRIL_TAGS = new int[] {
      2, 3, 4, 5, 8, 9, 10, 11, 15, 16
    };
    public static final int[] RED_TRENCH_APRIL_TAGS = new int[] {
      1, 12, 6, 7
    };
    public static final int[] RED_OUTPOST_APRIL_TAGS = new int[] {
      13, 14
    };
    public static final int[] BLUE_TOWER_APRIL_TAGS = new int[]{
      31, 32
    };
    public static final int[] BLUE_HUB_APRIL_TAGS = new int[]{
      18, 19, 20, 21, 24, 25, 26, 27
    };
    public static final int[] BLUE_TOWER_AND_HUB_APRIL_TAGS = new int[] {
      18, 19, 20, 21, 24, 25, 26, 27, 31, 32
    };
    public static final int[] BLUE_TRENCH_APRIL_TAGS = new int[] {
      17, 28, 22, 23
    };
    public static final int[] BLUE_OUTPOST_APRIL_TAGS = new int[] {
      29, 30
    };

    // TODO tune
    public static final Angle ROTATION_TOLERANCE = Degrees.of(3);

    // this is an initial guess lol
    // never changed
    // it works
    public static final double ROBOT_LATENCY = 0.0;

    public static final LinearVelocity[] OVER_RAMP_STAGE_MAX_SPEED = {
      MetersPerSecond.of(2.25),
      MetersPerSecond.of(1.67),
      MetersPerSecond.of(0.75)
    };
    public static final Distance OVER_RAMP_POSITION_TOLERANCE = Meters.of(0.2);
    public static final Angle OVER_RAMP_HEADING_TOLERANCE = Radians.of(20);

    private static final double RED_RAMP_BASE_X = 11.938;
    private static final double BLUE_RAMP_BASE_X = 4.6482;
    private static final double RAMP_LOW_Y = 2.498344;
    private static final double RAMP_HIGH_Y = 5.546344;

    /*
     * From alliance zone
     */
    private static final double AZ_POS_A_OFFSET_RED_X = 0.908304; // good
    private static final double AZ_POS_A_OFFSET_BLUE_X = -0.986536; // good
    private static final double AZ_POS_B_OFFSET_X = 0.1524 + 0.31; // good
    private static final double AZ_POS_C_OFFSET_X = 1.0668+0.1524; // good

    public static final Translation2d[] AZ_rampRed1 = new Translation2d[] {
      new Translation2d(RED_RAMP_BASE_X + AZ_POS_A_OFFSET_RED_X,   RAMP_LOW_Y),
      new Translation2d(RED_RAMP_BASE_X - AZ_POS_B_OFFSET_X,       RAMP_LOW_Y),
      new Translation2d(RED_RAMP_BASE_X - AZ_POS_C_OFFSET_X,       RAMP_LOW_Y)
    };
    public static final Translation2d[] AZ_rampRed2 = new Translation2d[] {
      new Translation2d(RED_RAMP_BASE_X + AZ_POS_A_OFFSET_RED_X,   RAMP_HIGH_Y),
      new Translation2d(RED_RAMP_BASE_X - AZ_POS_B_OFFSET_X,       RAMP_HIGH_Y),
      new Translation2d(RED_RAMP_BASE_X - AZ_POS_C_OFFSET_X,       RAMP_HIGH_Y)
    };
    public static final Translation2d[] AZ_rampBlue1 = new Translation2d[] {
      new Translation2d(BLUE_RAMP_BASE_X + AZ_POS_A_OFFSET_BLUE_X, RAMP_LOW_Y),
      new Translation2d(BLUE_RAMP_BASE_X + AZ_POS_B_OFFSET_X,      RAMP_LOW_Y),
      new Translation2d(BLUE_RAMP_BASE_X + AZ_POS_C_OFFSET_X,      RAMP_LOW_Y)
    };
    public static final Translation2d[] AZ_rampBlue2 = new Translation2d[] {
      new Translation2d(BLUE_RAMP_BASE_X + AZ_POS_A_OFFSET_BLUE_X, RAMP_HIGH_Y),
      new Translation2d(BLUE_RAMP_BASE_X + AZ_POS_B_OFFSET_X,      RAMP_HIGH_Y),
      new Translation2d(BLUE_RAMP_BASE_X + AZ_POS_C_OFFSET_X,      RAMP_HIGH_Y)
    };

    public static final Translation2d[] AZ_RAMP_POSA_CANDIDATES = new Translation2d[] {
      AZ_rampRed1[0], AZ_rampRed2[0], AZ_rampBlue1[0], AZ_rampBlue2[0]
    };

    /*
     * From neutral zone
     */
    private static final double NZ_POS_A_OFFSET_RED_X = AZ_POS_A_OFFSET_RED_X + 0.1143; // good
    private static final double NZ_POS_A_OFFSET_BLUE_X = AZ_POS_A_OFFSET_BLUE_X + 0.1143; // good
    private static final double NZ_POS_B_OFFSET_X = AZ_POS_B_OFFSET_X + 0.1143; // good
    private static final double NZ_POS_C_OFFSET_X = AZ_POS_C_OFFSET_X + 0.1143; // good

    public static final Translation2d[] NZ_rampRed1 = new Translation2d[] {
      new Translation2d(RED_RAMP_BASE_X - NZ_POS_A_OFFSET_RED_X,   RAMP_LOW_Y),
      new Translation2d(RED_RAMP_BASE_X + NZ_POS_B_OFFSET_X,       RAMP_LOW_Y),
      new Translation2d(RED_RAMP_BASE_X + NZ_POS_C_OFFSET_X,       RAMP_LOW_Y)
    };
    public static final Translation2d[] NZ_rampRed2 = new Translation2d[] {
      new Translation2d(RED_RAMP_BASE_X - NZ_POS_A_OFFSET_RED_X,   RAMP_HIGH_Y),
      new Translation2d(RED_RAMP_BASE_X + NZ_POS_B_OFFSET_X,       RAMP_HIGH_Y),
      new Translation2d(RED_RAMP_BASE_X + NZ_POS_C_OFFSET_X,       RAMP_HIGH_Y)
    };
    public static final Translation2d[] NZ_rampBlue1 = new Translation2d[] {
      new Translation2d(BLUE_RAMP_BASE_X - NZ_POS_A_OFFSET_BLUE_X, RAMP_LOW_Y),
      new Translation2d(BLUE_RAMP_BASE_X - NZ_POS_B_OFFSET_X,      RAMP_LOW_Y),
      new Translation2d(BLUE_RAMP_BASE_X - NZ_POS_C_OFFSET_X,      RAMP_LOW_Y)
    };
    public static final Translation2d[] NZ_rampBlue2 = new Translation2d[] {
      new Translation2d(BLUE_RAMP_BASE_X - NZ_POS_A_OFFSET_BLUE_X, RAMP_HIGH_Y),
      new Translation2d(BLUE_RAMP_BASE_X - NZ_POS_B_OFFSET_X,      RAMP_HIGH_Y),
      new Translation2d(BLUE_RAMP_BASE_X - NZ_POS_C_OFFSET_X,      RAMP_HIGH_Y)
    };

    public static final Translation2d[] NZ_RAMP_POSA_CANDIDATES = new Translation2d[] {
      NZ_rampRed1[0], NZ_rampRed2[0], NZ_rampBlue1[0], NZ_rampBlue2[0]
    };
  }

  public static class Field {
    public static final double BLUE_ZONE_X = 3.964;
    public static final double RED_ZONE_X = 12.549;
    public static final LoggedNetworkNumber MAX_BALL_Y_POS
      = new LoggedNetworkNumber("Tuning/maxBallYPos", 2.8);
      // TODO for comp
      // = new LoggedNetworkNumber("Tuning/maxBallYPos", 2.7);
    public static final double HUB_Y_POS = 1.83;
    public static final double GRAVITY_VALUE = 9.80665;
    public static final DoubleSupplier HUB_HANG_TIME =
      () -> (
        Math.sqrt((MAX_BALL_Y_POS.getAsDouble() - Shooter.SHOOTER_OFFSET_Z.in(Meters)) * 2) +
        Math.sqrt(
          2 * (
            (MAX_BALL_Y_POS.getAsDouble() - Shooter.SHOOTER_OFFSET_Z.in(Meters)) -
            (HUB_Y_POS - Shooter.SHOOTER_OFFSET_Z.in(Meters))
          )
        )
      ) / Math.sqrt(GRAVITY_VALUE);
    // we're only playing on andymark field so we can just use these
    public static final Distance FIELD_X = Inches.of(650.12);
    public static final Distance FIELD_Y = Inches.of(316.64);
    public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_X.div(2), FIELD_Y.div(2));
    public static final Translation2d BLUE_HUB_COORDINATES = new Translation2d(4.619, 4.049);
    public static final Translation2d RED_HUB_COORDINATES = new Translation2d(11.925, 4.049);

    public static final Pose2d BLUE_TOWER =
        new Pose2d(
            new Translation2d(Meters.of(1.6), Meters.of(3.7)), Rotation2d.fromDegrees(0));

    public static final Pose2d BLUE_TOWER_DEPOT_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(1.53), Meters.of(3.918)), Rotation2d.fromDegrees(0));  
    public static final Pose2d BLUE_TOWER_OUTPOST_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(1.53), Meters.of(3.030)), Rotation2d.fromDegrees(0));  
    public static final Pose2d BLUE_TOWER_DEPOT_ALIGN_POSE =
        new Pose2d(
            new Translation2d(Meters.of(2), Meters.of(3.918)), Rotation2d.fromDegrees(0));  
    public static final Pose2d BLUE_TOWER_OUTPOST_ALIGN_POSE =
        new Pose2d(
            new Translation2d(Meters.of(2), Meters.of(3.030)), Rotation2d.fromDegrees(0));

    public static final Pose2d RED_TOWER =
        new Pose2d(
            new Translation2d(Meters.of(14.919), Meters.of(4.299)), Rotation2d.fromDegrees(180));

    public static final Pose2d RED_TOWER_DEPOT_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(14.919), Meters.of(4.2)), Rotation2d.fromDegrees(180));  
    public static final Pose2d RED_TOWER_OUTPOST_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(14.919), Meters.of(5.1)), Rotation2d.fromDegrees(180));  
   public static final Pose2d RED_TOWER_DEPOT_ALIGN_POSE =
        new Pose2d(
            new Translation2d(Meters.of(12.919), Meters.of(4.2)), Rotation2d.fromDegrees(180));  
    public static final Pose2d RED_TOWER_OUTPOST_ALIGN_POSE =
        new Pose2d(
            new Translation2d(Meters.of(12.919), Meters.of(5.1)), Rotation2d.fromDegrees(180));  

    public static final Translation2d BLUE_BOTTOM_PASS_COORDINATES = new Translation2d(1.5, 2.235);
    public static final Translation2d BLUE_TOP_PASS_COORDINATES = new Translation2d(BLUE_BOTTOM_PASS_COORDINATES.getX(), 6.235);
    public static final Translation2d RED_BOTTOM_PASS_COORDINATES = new Translation2d(FIELD_X.minus(BLUE_BOTTOM_PASS_COORDINATES.getMeasureX()).in(Meters), BLUE_BOTTOM_PASS_COORDINATES.getY());
    public static final Translation2d RED_TOP_PASS_COORDINATES = new Translation2d(RED_BOTTOM_PASS_COORDINATES.getX(), BLUE_TOP_PASS_COORDINATES.getY());

    // meters
    public static final double HALF_FIELD_Y_POS = 4.022;
    
  }

  public static class Intake {
    public static final int INTAKE_MOTOR_LEADER_ID = 40;
    public static final int ARM_MOTOR_ID = 41;
    public static final int ARM_ENCODER_ID = 42;
    public static final int INTAKE_MOTOR_FOLLOWER_ID = 43;

    public static final Temperature OVERHEATING_TEMP = Celsius.of(100);

    // volts
    public static final double INTAKE_SPEED = 12;

    public static final double JIGGLE_TOLERANCE = 0.2;

    public static final Angle STOW_ANGLE = Rotations.of(-.07);
    public static final Angle DEPLOY_ANGLE = Rotations.of(0.2478);
    // straight up jiggling it
    public static final Angle JIGGLE_ANGLE = Rotations.of(0.15);
    public static final Angle HIGH_JIGGLE_ANGLE = Rotations.of(0.15);

    // Tolerance of check for arm being deployed
    // percent based on deploy angle
    // 5% probably reasonable for now
    public static final Angle DEPLOY_TOLERANCE = Degrees.of(15);
  }

  public static class LED {
    // TODO update
    public static final int CANDLE_ID = 60;

    // TODO update
    public static final int START_INDEX = 8;
    public static final int END_INDEX = 77;

    // #663399
    public static final RGBWColor DEFAULT_COLOR = new RGBWColor(102, 51, 153);
    // #00FF00
    public static final RGBWColor ACTIVE_COLOR = new RGBWColor(0, 255, 0);
    // #AD03DE
    public static final RGBWColor INACTIVE_COLOR = new RGBWColor(173, 3, 222);
    // #FFFD01
    public static final RGBWColor WARNING_COLOR = new RGBWColor(255, 253, 1);
    // #FF0000
    public static final RGBWColor RED_COLOR = new RGBWColor(255, 0, 0);
    // #0000FF
    public static final RGBWColor BLUE_COLOR = new RGBWColor(0, 0, 255);
    // #FFFFFF
    public static final RGBWColor WHITE_COLOR = new RGBWColor(255, 255, 255);
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_QUADRANT_CHOOSER_NAME = "Auto Start Position";
    public static final String SMARTDASHBOARD_AUTO_TYPE_CHOOSER_NAME = "Auto Selector";
  }
}
