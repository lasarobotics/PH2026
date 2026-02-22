package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.lasarobotics.fsm.SystemState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoHoncho {
  public enum BasicShootAuto implements SystemState {
    START {
      Pose2d targetPose;

      @Override
      public void initialize() {
        targetPose = new Pose2d(
          new Translation2d(
            2.5,
            1.5
          ),
          new Rotation2d(
            Degrees.of(45)
          )
        );
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          targetPose,
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public void end(boolean interrupted) {
        DriveSubsystem.getInstance().stopMoving();
      }

      @Override
      public SystemState nextState() {
        if (
          DriveSubsystem.atDestination(
            targetPose, 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return SHOOT;
        
        return this;
      }
    },
    SHOOT {
      @Override
      public void initialize() {
        s_wantToShoot = true;
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToShoot = false;
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    }
  }

  private static boolean s_wantToShoot = false;

  public static boolean autoWantToShoot() {
    return s_wantToShoot;
  }
}
