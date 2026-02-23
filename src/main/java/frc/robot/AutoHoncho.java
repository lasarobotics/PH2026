package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.lasarobotics.fsm.SystemState;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoHoncho {  
  public static AutoPositionConfig positionConfig;

  public enum BasicShootAuto implements SystemState {
    START {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AllianceZoneMiddleSide(),
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
            positionConfig.AllianceZoneMiddleSide(), 
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

  public enum ShootAndClimbAuto implements SystemState {
     START {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AllianceZoneMiddleSide(),
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
            positionConfig.AllianceZoneMiddleSide(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return SHOOT;
        
        return this;
      }
    },
    SHOOT {
      Timer timer = new Timer();

      @Override
      public void initialize() {
        s_wantToShoot = true;

        timer.start();
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToShoot = false;
      }

      @Override
      public SystemState nextState() {
        if (timer.hasElapsed(Constants.Auto.EightBallShootingTime)) {
          return GO_TO_CLIMB;
        }
        return this;
      }
    },
    GO_TO_CLIMB {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.TowerPose(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }
      @Override
      public SystemState nextState() {
        if (
          DriveSubsystem.atDestination(
            positionConfig.TowerPose(),
            Constants.Auto.LOW_DISTANCE_TOLERANCE, 
            Constants.Auto.LOW_ROTATION_TOLERANCE
          )
        ) return SHOOT; //TODO: should return climb
        
        return this;
      }

      @Override
      public void end(boolean interrupted) {
        DriveSubsystem.getInstance().stopMoving();
      }
    },
  }

  private static boolean s_wantToShoot = false;

  public static boolean autoWantToShoot() {
    return s_wantToShoot;
  }
}
