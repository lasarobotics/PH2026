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

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  public static class Auto {
    public static final double EightBallShootingTime = 5;

    public static final Distance HIGH_DISTANCE_TOLERANCE = Meters.of(0.15);
    public static final Distance LOW_DISTANCE_TOLERANCE = Meters.of(0.025);

    public static final Angle HIGH_ROTATION_TOLERANCE = Degrees.of(2.5);
    public static final Angle LOW_ROTATION_TOLERANCE = Degrees.of(0.5); 

    public static final Pose2d BLUE_RIGHT_POSE = new Pose2d(
      new Translation2d(
        2, 1.5
      ),
      new Rotation2d(
        Degrees.of(45)
      )
    );
    public static final Pose2d BLUE_LEFT_POSE = new Pose2d(
      new Translation2d(
        2.5, 6.5
      ),
      new Rotation2d(
        Degrees.of(-45)
      )
    );
    public static final Pose2d BLUE_TOWER_SHOOTING_POSE = new Pose2d(
      new Translation2d(
        1.7,
        4.05
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );  
    public static final Pose2d BLUE_CENTER_POSE = new Pose2d(
      new Translation2d(
        2, 4
      ),
      new Rotation2d(
        Degrees.of(0)
      )
    );
    public static final Pose2d BLUE_DEPOT_ENTER_POSE = new Pose2d(
      new Translation2d(
        0.5,
        7
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
    public static final Pose2d BLUE_DEPOT_EXIT_POSE = new Pose2d(
      new Translation2d(
        5.5,
        7
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
    public static final Pose2d RED_RIGHT_POSE = new Pose2d(
      new Translation2d(
        14.5, 6.5
      ),
      new Rotation2d(
        Degrees.of(-135)
      )
    );
    public static final Pose2d RED_LEFT_POSE = new Pose2d(
      new Translation2d(
        14.5, 1.5
      ),
      new Rotation2d(
        Degrees.of(135)
      )
    );
    public static final Pose2d RED_TOWER_SHOOTING_POSE = new Pose2d(
      new Translation2d(
        14.8,
        4.2
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
    public static final Pose2d RED_CENTER_POSE = new Pose2d(
      new Translation2d(
        14, 4
      ),
      new Rotation2d(
        Degrees.of(180)
      )
    );
    public static final Pose2d RED_DEPOT_ENTER_POSE = new Pose2d(
      new Translation2d(
        16,
        1
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );
    public static final Pose2d RED_DEPOT_EXIT_POSE = new Pose2d(
      new Translation2d(
        16,
        2.5
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );



    public static final Pose2d BLUE_LEFT_ABUMP_NZ_POSE = new Pose2d(
      new Translation2d(
        5.982,
        5.719
      ),
      new Rotation2d(
        Degrees.of(0)
      )
    );
    public static final Pose2d BLUE_LEFT_ABUMP_AZ_POSE = new Pose2d(
      new Translation2d(
        3.283,
        5.691
      ),
      new Rotation2d(
        Degrees.of(-45) // anti beaching, used to be 0 (Q48 Belton)
      )
    );
    public static final Pose2d BLUE_LEFT_DEPOT_NZ_POSE = new Pose2d(
      new Translation2d(
        7.7,
        6.878
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
    public static final Pose2d BLUE_LEFT_HUB_NZ_POSE = new Pose2d(
      new Translation2d(
        7.7,
        4.663 // changed after hitting trench Q48 Belton (used to be 4.478)
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
    public static final Pose2d BLUE_LEFT_OUTPOST_NZ_POSE = new Pose2d(
      new Translation2d(
        7.7,
        1.865
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );


    public static final Pose2d BLUE_RIGHT_ABUMP_NZ_POSE = new Pose2d(
      new Translation2d(
        5.982,
        2.596
      ),
      new Rotation2d(
        Degrees.of(0)
      )
    );
    public static final Pose2d BLUE_RIGHT_ABUMP_AZ_POSE = new Pose2d(
      new Translation2d(
        3.283,
        2.45
      ),
      new Rotation2d(
        Degrees.of(-45) // anti beaching, used to be 0 (Q48 Belton)
      )
    );
    public static final Pose2d BLUE_RIGHT_DEPOT_NZ_POSE = new Pose2d(
      new Translation2d(
        7.7,
        6.21
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );
    public static final Pose2d BLUE_RIGHT_HUB_NZ_POSE = new Pose2d(
      new Translation2d(
        7.7,
        3.5
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );
    public static final Pose2d BLUE_RIGHT_OUTPOST_NZ_POSE = new Pose2d(
      new Translation2d(
        7.7,
        1.865
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );


    public static final Pose2d RED_LEFT_ABUMP_NZ_POSE = new Pose2d(
      new Translation2d(
        10.64,
        2.596
      ),
      new Rotation2d(
        Degrees.of(0)
      )
    );
    public static final Pose2d RED_LEFT_ABUMP_AZ_POSE = new Pose2d(
      new Translation2d(
        13.2,
        2.18
      ),
      new Rotation2d(
        Degrees.of(-45) // anti beaching, used to be 0 (Q48 Belton)
      )
    );
    public static final Pose2d RED_LEFT_DEPOT_NZ_POSE = new Pose2d(
      new Translation2d(
        8.912,
        1.15
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );
    public static final Pose2d RED_LEFT_HUB_NZ_POSE = new Pose2d(
      new Translation2d(
        8.912,
        3.7
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );
    public static final Pose2d RED_LEFT_OUTPOST_NZ_POSE = new Pose2d(
      new Translation2d(
        8.912,
        5.86
      ),
      new Rotation2d(
        Degrees.of(90)
      )
    );


    public static final Pose2d RED_RIGHT_ABUMP_NZ_POSE = new Pose2d(
      new Translation2d(
        10.5,
        5.719
      ),
      new Rotation2d(
        Degrees.of(0)
      )
    );
    public static final Pose2d RED_RIGHT_ABUMP_AZ_POSE = new Pose2d(
      new Translation2d(
        13.134,
        5.691
      ),
      new Rotation2d(
        Degrees.of(-45) // anti beaching, used to be 0 (Q48 Belton)
      )
    );
    public static final Pose2d RED_RIGHT_OUTPOST_NZ_POSE = new Pose2d(
      new Translation2d(
        8.818,
        6.878
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
    public static final Pose2d RED_RIGHT_HUB_NZ_POSE = new Pose2d(
      new Translation2d(
        8.818,
        4.478
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
    public static final Pose2d RED_RIGHT_DEPOT_NZ_POSE = new Pose2d(
      new Translation2d(
        8.818,
        2.33
      ),
      new Rotation2d(
        Degrees.of(-90)
      )
    );
  }

  public static class Shooter {
    public static final int LEADER_SHOOTER_MOTOR_ID = 30;
    public static final int FOLLOWER_SHOOTER_ONE_MOTOR_ID = 31;
    public static final int FOLLOWER_SHOOTER_TWO_MOTOR_ID = 32;
    public static final int INDEXER_MOTOR_ID = 33;
    public static final int HOOD_MOTOR_ID = 34;
    public static final int HOOD_CANCODER_ID = 35;
    public static final int AGITATOR_MOTOR_ID = 36;

    // duty cycle
    // should be negative
    public static final double INDEXER_MOTOR_SPEED = -1.0;

    // set shooter motor to constant speed when not actively shooting
    // ideally, this will be somewhere in the
    // middle of how fast we generally shoot
    // 20 rotations per second is probably right because we
    // want to avoid being too far over
    public static final double SHOOTER_HOLD_SPEED = 20;

    // shooter speed tolerance is in rotations per second
    public static final double SHOOTER_SPEED_BELOW_TOLERANCE = 2;
    public static final double SHOOTER_SPEED_ABOVE_TOLERANCE = 2;
    public static final Angle HOOD_POSITION_TOLERANCE = Degrees.of(5);

    // how long we can be shooting after the end of the shift
    // this accounts for hang time:
    // basically we can shoot if time in shift - hangtime >= margin
    // so with 1.5s hangtime and 0s left in shift we have -1.5s
    // hub takes like 1 to 2 seconds to count & counts for 3 seconds after end of period
    // so -1.5 is probably alright
    public static final double SHOOTER_TIME_MARGIN = -1.5;

    public static final Distance SHOOTER_RADIUS = Inches.of(2);
    public static final double FLYWHEEL_RADIUS = 0.0508;

    // Note: using wpilib coordinate system
    // intake is front
    // x offset from center (assume middle of shooter)
    // (behind center)
    public static final Distance SHOOTER_OFFSET_X = Inches.of(-3.062500); // measured in cad
    // y offset from center
    // (right of center)
    public static final Distance SHOOTER_OFFSET_Y = Inches.of(-7.710630); // measured in cad
    // z offset from floor (hood at 25deg, center of ball exit height)
    public static final Distance SHOOTER_OFFSET_Z = Inches.of(23.422254); // measured in cad
    public static final Distance SHOOTER_DISTANCE_FROM_CENTER = Meters.of(
      Math.sqrt(
        Math.pow(SHOOTER_OFFSET_X.in(Meters), 2) + Math.pow(SHOOTER_OFFSET_Y.in(Meters), 2)
      )
    );

    // these values are for when the robot is pressed againt the tower
    // public static final Angle DUMB_HOOD_POSITION = Rotations.of(-0.06);
    public static final LoggedNetworkNumber DUMB_HOOD_POSITION
      = new LoggedNetworkNumber("/Tuning/dumbHoodPosition", 80 - 21.6);
    // In rotations per second
    // currently based on empirical measurement
    // public static final double DUMB_SHOOTER_SPEED = 34.5;
    // public static final double DUMB_SHOOTER_SPEED = 34.25;
    // public static final double DUMB_SHOOTER_SPEED = 33.75;
    public static final LoggedNetworkNumber DUMB_SHOOTER_SPEED
      = new LoggedNetworkNumber("/Tuning/dumbHoodPosition", 33.75);

    public static LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_SCALAR
      = new LoggedNetworkNumber("/Tuning/aimutilSpeedScalar", 1);
    public static LoggedNetworkNumber AIMUTIL_SHOOTER_SPEED_ADDEND
      = new LoggedNetworkNumber("/Tuning/aimutilSpeedFudger", 0);
    public static LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_SCALAR
      = new LoggedNetworkNumber("/Tuning/hoodAngleScalar");
    public static LoggedNetworkNumber AIMUTIL_HOOD_ANGLE_ADDEND
      = new LoggedNetworkNumber("/Tuning/hoodAngleAddend");
  }

  public static class Drive {
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3); // TODO measure
    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(0.75); // TODO measure
    public static final AngularVelocity AUTO_NAV_MAX_ANGULAR_RATE =
      RadiansPerSecond.of(2.0); // ~115 deg/s — controllable for auto
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1); // TODO  measure
    // normally 0.3 and 1.0
    public static final double SLOW_SPEED_SCALAR = 0.15;
    public static final double FAST_SPEED_SCALAR = .75;

    public static final String SHOOTER_LIMELIGHT_NAME = "limelight-shooter";
    public static final String CLIMB_LIMELIGHT_NAME = "limelight-climb";
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
    public static final double ROTATION_TOLERANCE = 0.05;

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

    public static final double INTAKE_SPEED = 1.0;

    public static final double JIGGLE_TOLERANCE = 0.2;

    // preliminary values
    public static final Angle STOW_ANGLE = Rotations.of(0.0); // TODO change maybe
    // might want to reduce
    // this should be basically the value of the soft limit of the arm
    public static final Angle DEPLOY_ANGLE = Rotations.of(0.267);
    // straight up jiggling it
    public static final Angle JIGGLE_ANGLE = Rotations.of(0.125);

    // Tolerance of check for arm being deployed
    // percent based on deploy angle
    // 5% probably reasonable for now
    public static final Angle DEPLOY_TOLERANCE = Degrees.of(10);
  }


  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_QUADRANT_CHOOSER_NAME = "Auto Start Position";
    public static final String SMARTDASHBOARD_AUTO_TYPE_CHOOSER_NAME = "Auto Selector";
  }
}
