// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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

  public static class ShooterSubsystem {
    public static final int shooterMotorId = 0; // TODO
    public static final int indexerMotorId = 0; // TODO
    public static final int hoodMotorId = 0; // TODO

    // TODO find value for this
    public static final double indexerMotorSpeed = 0;

    // TODO find value for this
    // set shooter motor to constant speed
    // ideally, this will be somewhere in the
    // middle of how fast we generally shoot
    public static final double shooterHoldSpeed = 0;

    public static final double shooterSpeedTolerance = 0; // TODO

    public static final double shooterTimeMargin = 0; // TODO

    public static final double hoodToMotorRatio = 0; // TODO
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

    public static final Translation2d HUB_COORDINATES = new Translation2d(4.619, 4.049);

    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_ANGULAR_RATE.in(RadiansPerSecond),
          MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));
  }
}
