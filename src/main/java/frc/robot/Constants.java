// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
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
import edu.wpi.first.units.measure.Current;
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
    public static final int LEADER_SHOOTER_MOTOR_ID = 0; // TODO
    // NOTE: Slave motors should spin in the same direction
    // (opposite to that of the master)
    public static final int FOLLOWER_SHOOTER_ONE_MOTOR_ID = 0; // TODO
    public static final int FOLLOWER_SHOOTER_TWO_MOTOR_ID = 0; // TODO
    public static final int INDEXER_MOTOR_ID = 0; // TODO
    public static final int HOOD_MOTOR_ID = 0; // TODO

    // TODO find value for this
    // voltage
    public static final double INDEXER_MOTOR_SPEED = 0;

    // TODO find value for this
    // set shooter motor to constant speed
    // ideally, this will be somewhere in the
    // middle of how fast we generally shoot
    public static final double SHOOTER_HOLD_SPEED = 0;

    // TODO maybe negative? find value
    // This is the voltage that the hood goes at whole zeroing
    public static final double HOOD_ZERO_VOLTAGE = 1;
    // TODO find actual value
    // This should be something around the maximum current of the motor
    public static final Current HOOD_STALL_CURRENT = Amps.of(40);
    // TODO find value
    // Tolerance for detecting if hood motor has stalled
    public static final Current HOOD_ZERO_CURRENT_TOLERANCE = Amps.of(0.1);
    // TODO find value
    // Tolerance for detecting if hood speed is zero
    public static final double HOOD_ZERO_SPEED_TOLERANCE = 0.1;

    public static final double SHOOTER_SPEED_TOLERANCE = 0; // TODO
    public static final Angle HOOD_POSITION_TOLERANCE = Degrees.of(0); // TODO

    public static final double SHOOTER_TIME_MARGIN = 0; // TODO

    public static final double HOOD_TO_MOTOR_RATIO = 0; // TODO

    public static final Distance SHOOTER_RADIUS = Inches.of(2);
    public static final double FLYWHEEL_RADIUS = 0.0508;

    public static final double SHOOTER_OFFSET = 0.0; //TODO change

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

    public static final double ROTATION_TOLERANCE = 0; // TODO

    public static final double ROBOT_LATENCY = 0.15; //TODO: measure

    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_ANGULAR_RATE.in(RadiansPerSecond),
          MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  }

  public static class Field {
    public static final double MAX_BALL_Y_POS = 2.8;
    public static final double END_BALL_Y_POS = 1.83;
    public static final double GRAVITY_VALUE = 9.80665;
    public static final Translation2d HUB_COORDINATES = new Translation2d(4.619, 4.049);
    // TODO: Math looks off, I think parentheses should make it so that the sum of the roots is divided by root g -ck
    public static final double HANG_TIME = (((
      Math.sqrt(Constants.Field.MAX_BALL_Y_POS)) * 2) + 
      Math.sqrt(Constants.Field.END_BALL_Y_POS) / Math.sqrt(Constants.Field.GRAVITY_VALUE));
    public static final Pose2d BLUE_TOWER =
        new Pose2d(
            new Translation2d(Meters.of(1.6), Meters.of(3.7)), Rotation2d.fromDegrees(180));  
    public static final Pose2d RED_TOWER =
        new Pose2d(
            new Translation2d(Meters.of(14.919), Meters.of(4.299)), Rotation2d.fromDegrees(180));  
    
  }

  public static class Vision {
    public static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera"; // NEED TO CHANGE maybe.....
    public static final int OBJECT_DETECTION_PIPELINE_INDEX = 0;
    public static final int MAX_TARGETS_TO_PROCESS = 60; // can be increased, js for now kept low to help with performance

    public static final double MIN_CONFIDENCE = 0.2;
    public static final double MIN_TARGET_AREA_PERCENT = 0.1;
    public static final double AREA_PERCENT_AT_ONE_METER = 12.0;
    public static final double BOUNDING_BOX_AREA_AT_ONE_METER = 9000.0;
    public static final double MIN_DISTANCE_METERS = Meters.of(0.05).in(Meters);
  }

  public static class Intake {
    // TODO: find the ids of the motors and encoder
    public static final int INTAKE_MOTOR_ID = 0;
    public static final int ARM_MOTOR_ID = 0;
    public static final int ARM_ENCODER_ID = 0;

    // TODO: find values of intake positions
    public static final Dimensionless INTAKE_SPEED = Value.of(0.6);
    public static final Angle STOW_ANGLE = Degrees.of(0);
    public static final Angle DEPLOY_ANGLE = Degrees.of(0);
  }

  public static class Hopper {
    // TODO find these
    public static final int CANRANGE_TOP_ONE_ID = 0;
    public static final int CANRANGE_TOP_TWO_ID = 0;
    public static final int CANRANGE_TOP_THREE_ID = 0;
    public static final int CANRANGE_BOTTOM_ONE_ID = 0;
    public static final int CANRANGE_BOTTOM_TWO_ID = 0;
    public static final int CANRANGE_BOTTOM_THREE_ID = 0;

    // TODO update to be not 5 inches
    public static final Distance BLOCKED_DISTANCE = Inches.of(5);

    // TODO adjust maybe
    public static final Time DELAY_TIME = Seconds.of(2);
  }

  public static class Climb {
    // TODO: find the ids of the climb motors and encoder
    public static final int CLIMB_MOTOR_1_ID = 0;
    public static final int CLIMB_MOTOR_2_ID = 0;
    public static final int ARM_ENCODER_ID = 0;

    // TODO: Find the values of climb constants
    public static final double CLIMB_SPEED = 0.0;
    public static final double CLIMB_SPEED_SLOW = CLIMB_SPEED * 0.2;
    public static final Angle CLIMB_ANGLE = Degrees.of(0);
    public static final Angle STOW_ANGLE = Degrees.of(0);
  }
}
