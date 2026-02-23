// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
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

  public static class Shooter {
    public static final int LEADER_SHOOTER_MOTOR_ID = 30;
    public static final int FOLLOWER_SHOOTER_ONE_MOTOR_ID = 31;
    public static final int FOLLOWER_SHOOTER_TWO_MOTOR_ID = 32;
    public static final int INDEXER_MOTOR_ID = 33;
    public static final int HOOD_MOTOR_ID = 34;
    public static final int HOOD_CANCODER_ID = 35;

    // TODO find value for this
    // voltage
    public static final double INDEXER_MOTOR_SPEED = 0;

    // TODO find value for this
    // set shooter motor to constant speed
    // ideally, this will be somewhere in the
    // middle of how fast we generally shoot
    // shooter spreadsheet indicates that 2800
    // is about right (for ymax = 2.5m)
    // https://docs.google.com/spreadsheets/d/1W-cpAlIJaHPbepHAKy7-uNkREk7_ITiVmTQvAB0_ZDM/edit
    public static final double SHOOTER_HOLD_SPEED = 0;

    public static final double SHOOTER_SPEED_TOLERANCE = 0; // TODO
    public static final Angle HOOD_POSITION_TOLERANCE = Degrees.of(1); // TODO

    public static final double SHOOTER_TIME_MARGIN = 0; // TODO

    public static final double HOOD_TO_MOTOR_RATIO = 0; // TODO

    public static final Distance SHOOTER_RADIUS = Inches.of(2);
    public static final double FLYWHEEL_RADIUS = 0.0508;

    // x offset from center
    public static final Distance SHOOTER_OFFSET_X = Inches.of(7.710630); // measured in cad
    // y offset from floor (hood at 25deg, center of ball exit height)
    public static final Distance SHOOTER_OFFSET_Y = Inches.of(23.422254); // measured in cad

    // TODO find actual value
    // Probably going to be pretty high? Unknown
    public static final Angle DUMB_HOOD_POSITION = Degrees.of(80);
    // TODO find actual value
    public static final double DUMB_SHOOTER_SPEED = 0;
  }

  public static class Drive {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3); // TODO measure
    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(0.75); // TODO measure
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1); // TODO  measure
    public static final double SLOW_SPEED_SCALAR = 0.3;
    public static final double FAST_SPEED_SCALAR = 1.0;

    public static final String SHOOTER_LIMELIGHT_NAME = "shooter";
    public static final String CLIMB_LIMELIGHT_NAME = "climb";
    public static final String BACK_LIMELIGHT_NAME = "back";

    public static final int THROTTLE_OFF = 200;
    public static final int THROTTLE_IDLE = 25;
    public static final int THROTTLE_RUNNING = 0;

    public static final int[] ALL_APRIL_TAGS = new int[]{};
    public static final int[] RED_TOWER_APRIL_TAGS = new int[]{
      15, 16
    };
    public static final int[] RED_HUB_APRIL_TAGS = new int[]{
      2, 3, 4, 5, 8, 9, 10, 11
    };
    public static final int[] RED_TOWER_AND_HUB_APRIL_TAGS = new int[] {
      9, 10, 15, 16
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
      25, 26, 31, 32
    };
    public static final int[] BLUE_TRENCH_APRIL_TAGS = new int[] {
      17, 28, 22, 23
    };
    public static final int[] BLUE_OUTPOST_APRIL_TAGS = new int[] {
      29, 30
    };

    // This is used as a percent tolerance
    // 5% seems reasonable, subject to change
    public static final double ROTATION_TOLERANCE = 0.05; // TODO

    public static final double ROBOT_LATENCY = 0.15; //TODO: measure

    public static final TrapezoidProfile.Constraints TRANSLATE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        MAX_SPEED.in(MetersPerSecond),
        MAX_ACCELERATION.in(MetersPerSecondPerSecond)
      );

    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
        MAX_ANGULAR_RATE.in(RadiansPerSecond),
        MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond)
      );

    // TODO: change offsets according to 418 wheels
    public static final double EPSILON = 0.000001;

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
    public static final double MAX_BALL_Y_POS = 2.8;
    public static final double HUB_Y_POS = 1.83;
    public static final double GRAVITY_VALUE = 9.80665;
    public static final Translation2d BLUE_HUB_COORDINATES = new Translation2d(4.619, 4.049);
    public static final Translation2d RED_HUB_COORDINATES = new Translation2d(11.925, 4.049);

    public static final Pose2d BLUE_TOWER =
        new Pose2d(
            new Translation2d(Meters.of(1.6), Meters.of(3.7)), Rotation2d.fromDegrees(-90));  
    public static final Pose2d BLUE_TOWER_DEPOT_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(1.6), Meters.of(3.9)), Rotation2d.fromDegrees(-90));  
    public static final Pose2d BLUE_TOWER_OUTPOST_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(1.6), Meters.of(2.954)), Rotation2d.fromDegrees(-90));  
    public static final Pose2d RED_TOWER =
        new Pose2d(
            new Translation2d(Meters.of(14.919), Meters.of(4.299)), Rotation2d.fromDegrees(90)); 
    public static final Pose2d RED_TOWER_DEPOT_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(14.919), Meters.of(4.2)), Rotation2d.fromDegrees(90));  
    public static final Pose2d RED_TOWER_OUTPOST_SIDE =
        new Pose2d(
            new Translation2d(Meters.of(14.919), Meters.of(5.1)), Rotation2d.fromDegrees(90));  

    
    public static final Translation2d RED_BOTTOM_PASS_COORDINATES = new Translation2d(14.231, 2.235);
    public static final Translation2d RED_TOP_PASS_COORDINATES = new Translation2d(14.231, 6.235);
    public static final Translation2d BLUE_BOTTOM_PASS_COORDINATES = new Translation2d(2.051, 2.235);
    public static final Translation2d BLUE_TOP_PASS_COORDINATES = new Translation2d(2.051, 6.235);

    // meters
    public static final double HALF_FIELD_Y_POS = 4.022;
    
  }

  public static class Intake {
    public static final int INTAKE_MOTOR_ID = 40;
    public static final int ARM_MOTOR_ID = 41;
    public static final int ARM_ENCODER_ID = 42;

    // TODO: find values of intake positions
    public static final Dimensionless INTAKE_SPEED = Value.of(0.6);

    // preliminary values
    public static final Angle STOW_ANGLE = Rotations.of(0);
    // might want to reduce
    // this should be basically the value of the soft limit of the arm
    // TODO update for bumper
    public static final Angle DEPLOY_ANGLE = Rotations.of(.28);

    // Tolerance of check for hopper being deployed
    // percent based on deploy angle
    // 5% probably reasonable for now
    public static final double DEPLOY_TOLERANCE = 0.05; // TODO
  }

  public static class Hopper {
    public static final int CANRANGE_TOP_ONE_ID = 50;
    public static final int CANRANGE_TOP_TWO_ID = 51;
    public static final int CANRANGE_BOTTOM_ID = 52;

    // 18 inches is wall to wall where bottom canrange is
    // so 17 inches for extra tolerance
    public static final Distance BOTTOM_BLOCKED_DISTANCE = Inches.of(17);
    // 32.5 inches from climber tube to hopper wall when fully extended
    // We want a bunch of margin & this is for like fully full
    // so 29 is probably alright
    // TODO double check
    public static final Distance TOP_BLOCKED_DISTANCE_OPEN = Inches.of(29);
    // 17 inches from climb tube to approx. hopper closed position
    // so 14 probably good
    // TODO double check
    public static final Distance TOP_BLOCKED_DISTANCE_CLOSED = Inches.of(14);

    // TODO adjust maybe
    public static final Time DELAY_TIME = Seconds.of(0.5);
  }

  public static class Climb {
    public static final int CLIMB_MOTOR_1_ID = 60;
    public static final int CLIMB_MOTOR_2_ID = 61;
    public static final int ARM_ENCODER_ID = 62;

    // TODO: Find out where the servo is
    public static final int SERVO_CHANNEL = 0;

    public static final double CLIMB_SPEED_RPS = 3000.0 / 60.0;
    public static final Angle CLIMB_ANGLE = Degrees.of(70);
    public static final Angle DEPLOY_ANGLE = Degrees.of(95);
    public static final Angle STOW_ANGLE = Degrees.of(0);
    public static final Angle CLIMB_TOLERANCE = Degrees.of(2);

    // TODO: Find the values of climb servo constants
    public static final Angle SERVO_RETRACT_ANGLE = Degrees.of(0);
    public static final Angle SERVO_STOW_ANGLE = Degrees.of(0);
  }

  public static class Auto {
    public static final Distance HIGH_DISTANCE_TOLERANCE = Meters.of(0.2);
    public static final Distance LOW_DISTANCE_TOLERANCE = Meters.of(0.05);

    public static final Angle HIGH_ROTATION_TOLERANCE = Degrees.of(20);
    public static final Angle LOW_ROTATION_TOLERANCE = Degrees.of(2);
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_CLIMB_CHOOSER_NAME = "Auto Climb Position";
  }
}
