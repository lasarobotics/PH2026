package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoPositionConfig.Quadrant;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoHoncho extends StateMachine implements AutoCloseable {  
  public static AutoPositionConfig positionConfig;
  public static SystemState startingState;

  public enum NothingAuto implements SystemState {
    NOTHING {
      @Override
      public SystemState nextState() {
        return this;
      }
    }
  }

  public enum BasicShootAuto implements SystemState {
    START {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AllianceZoneSide(),
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
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AllianceZoneSide(), 
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
        DriveSubsystem.getInstance().driveAutoAim();
        s_wantToDumbShoot = true;
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToDumbShoot = false;
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        return this;
      }
    }
  }

  public enum ShootAndClimbAuto implements SystemState {
    START {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AllianceZoneSide(),
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
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AllianceZoneSide(), 
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
        s_wantToDumbShoot = true;
        DriveSubsystem.getInstance().driveAutoAim();
        timer.start();
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToDumbShoot = false;
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (timer.hasElapsed(Constants.Auto.EightBallShootingTime)) {
          return GO_TO_CLIMB;
        }

        return this;
      }
    },
    GO_TO_CLIMB {
      Pose2d autoClimbPosition;

      @Override
      public void initialize() {
        DriveSubsystem.getInstance().driverControl();
        autoClimbPosition = DriveSubsystem.s_climbPosition;
        ClimbSubsystem.getInstance().deploy();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.TowerPose(),
          autoClimbPosition,
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
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            // positionConfig.TowerPose(),
            autoClimbPosition,
            Constants.Auto.LOW_DISTANCE_TOLERANCE, 
            Constants.Auto.LOW_ROTATION_TOLERANCE
          )
        ) return CLIMB;

        return this;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().stopIntake();
        ShooterSubsystem.getInstance().stopOperation();
      }

      @Override
      public void execute() {
        if (ClimbSubsystem.getInstance().inDeployPosition()) {
          Logger.recordOutput("HeadHoncho/executeClimbButtonReady", true);
          ClimbSubsystem.getInstance().climb();
        } else {
          Logger.recordOutput("HeadHoncho/executeClimbButtonReady", false);
        }
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        // TODO: Leave this state? Auto should end with climb...
        return this;
      }
    },
  }

  // this is more of a POC/example than actual auto
  public enum CrossRampAuto implements SystemState {
    START {
      @Override
      public void initialize() {
        DriveSubsystem.getInstance().driveOverRamp();
        s_wantToCrossBump = true;
      }

      @Override
      public void execute() {
        if (DriveSubsystem.getInstance().getState()
            != DriveSubsystem.DriveStates.OVER_RAMP) {
          s_wantToCrossBump = false;
        }
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;
        
        return this;
      }
    }
  }

  private static boolean s_wantToShoot = false;
  private static boolean s_wantToDumbShoot = false;
  private static boolean s_wantToCrossBump = false;

  public static SendableChooser<String> s_autoQuadrantChooser = new SendableChooser<>();
  public static SendableChooser<String> s_autoTypeChooser = new SendableChooser<>();

  public static void setAutoQuadrant(String quadrant) {
    Logger.recordOutput("AutoHoncho/setAutoQuadrant", quadrant);
    Quadrant quad = null;
    switch (quadrant) {
      case "Blue Left":
        quad = Quadrant.BLUE_LEFT;
        break;
      case "Blue Right":
        quad = Quadrant.BLUE_RIGHT;
        break;
      case "Red Left":
        quad = Quadrant.RED_LEFT;
        break;
      case "Red Right":
        quad = Quadrant.RED_RIGHT;
        break;
      default:
        break;
    }
    if (quad != null) {
      positionConfig = new AutoPositionConfig(quad);
    }
  }

  public static void setAutoType(String type) {
    Logger.recordOutput("AutoHoncho/setAutoType", type);
    switch (type) {
      case "Nothing":
        startingState = NothingAuto.NOTHING;
        break;
      case "Basic Shoot":
        startingState = BasicShootAuto.START;
        break;
      case "Shoot and Climb":
        startingState = ShootAndClimbAuto.START;
        break;
    }
  }

  public AutoHoncho() {
    super(startingState);
    if (startingState == null) {
      Logger.recordOutput("AutoHoncho/creationAutoType", "null");
    } else {
      Logger.recordOutput("AutoHoncho/creationAutoType", startingState.toString());
    }
  }

  public void periodic() {
    Logger.recordOutput(getName() + "/currentState", getState().toString());
  }

  public static boolean autoWantToShoot() {
    return s_wantToShoot;
  }

  public static boolean autoWantToDumbShoot() {
    return s_wantToDumbShoot;
  }

  public static boolean autoWantToCrossRamp() {
    return s_wantToCrossBump;
  }

  @Override
  public void close() {}
}
