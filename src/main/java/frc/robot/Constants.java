// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
}
