// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
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

    public static final double ROBOT_LATENCY = 0.15; //TODO: measure

    public static final Translation2d HUB_COORDINATES = new Translation2d(4.619, 4.049);

    public static final double HANG_TIME = (((
      Math.sqrt(Constants.Field.MAX_BALL_Y_POS)) * 2) + 
      Math.sqrt(Constants.Field.END_BALL_Y_POS) /Math.sqrt(Constants.Field.GRAVITY_VALUE));

    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_ANGULAR_RATE.in(RadiansPerSecond),
          MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

    public static final double SHOOTER_OFFSET = 0.0; //TODO change

    public static final double FUEL_ALIGN_SPEED_SCALAR = 0.15;
    public static final double FUEL_ALIGN_STOP_DISTANCE_METERS = 0.10; // stop about 10 cm away CAN BE CHANGED js guestimating on intake rn
    public static final double FUEL_ALIGN_DISTANCE_KP = 1.0;
    public static final double GO_TO_POSA_SPEED_SCALAR = 1.0;
    public static final double GO_OVER_BUMP_SPEED_SCALAR = 1.0;
    public static final double GO_DOWN_BUMP_SPEED_SCALAR = 1.0;
    // bump pos should go below
    public static final Pose2d AZ_bumpRed1_posa = new Pose2d(12.846304, 2.498344, new Rotation2d(Math.PI / 4));
    public static final Pose2d AZ_bumpRed2_posa = new Pose2d(12.846304, 5.546344, new Rotation2d(Math.PI / 4));
    public static final Pose2d AZ_bumpBlue1_posa = new Pose2d(3.661664, 2.498344, new Rotation2d(Math.PI / 4));
    public static final Pose2d AZ_bumpBlue2_posa = new Pose2d(3.661664, 5.546344, new Rotation2d(Math.PI / 4));

    public static final Pose2d AZ_bumpRed1_posb = new Pose2d(11.938, 2.498344, new Rotation2d(Math.PI / 4));
    public static final Pose2d AZ_bumpRed2_posb = new Pose2d(11.938, 5.546344, new Rotation2d(Math.PI / 4));
    public static final Pose2d AZ_bumpBlue1_posb = new Pose2d(4.6482, 2.498344, new Rotation2d(Math.PI / 4));
    public static final Pose2d AZ_bumpBlue2_posb = new Pose2d(4.6482, 5.546344, new Rotation2d(Math.PI / 4));

    public static final Pose2d AZ_bumpRed1_posc = new Pose2d(13.0048, 2.498344, new Rotation2d(0));
    public static final Pose2d AZ_bumpRed2_posc = new Pose2d(13.0048, 5.546344, new Rotation2d(0));
    public static final Pose2d AZ_bumpBlue1_posc = new Pose2d(5.715, 2.498344, new Rotation2d(0));
    public static final Pose2d AZ_bumpBlue2_posc = new Pose2d(5.715, 5.546344, new Rotation2d(0));

    // NZ has 1000 rn bc we are not testing that rn
    public static final Pose2d NZ_bumpRed1_posa = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));
    public static final Pose2d NZ_bumpRed2_posa = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));
    public static final Pose2d NZ_bumpBlue1_posa = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));
    public static final Pose2d NZ_bumpBlue2_posa = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));

    public static final Pose2d NZ_bumpRed1_posb = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));
    public static final Pose2d NZ_bumpRed2_posb = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));
    public static final Pose2d NZ_bumpBlue1_posb = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));
    public static final Pose2d NZ_bumpBlue2_posb = new Pose2d(1000, 1000, new Rotation2d(Math.PI / 4));

    public static final Pose2d NZ_bumpRed1_posc = new Pose2d(1000, 1000, new Rotation2d(0));
    public static final Pose2d NZ_bumpRed2_posc = new Pose2d(1000, 1000, new Rotation2d(0));
    public static final Pose2d NZ_bumpBlue1_posc = new Pose2d(1000, 1000, new Rotation2d(0));
    public static final Pose2d NZ_bumpBlue2_posc = new Pose2d(1000, 1000, new Rotation2d(0));

    public static final double FLYWHEEL_RADIUS = 0.0508;
  }

  public static class Field {
    public static final double MAX_BALL_Y_POS = 2.8;
    public static final double END_BALL_Y_POS = 1.83;
    public static final double GRAVITY_VALUE = 9.80665;
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
}
