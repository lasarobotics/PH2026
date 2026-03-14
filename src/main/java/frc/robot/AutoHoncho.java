package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoPositionConfig.Quadrant;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoHoncho extends StateMachine implements AutoCloseable {  
  public static AutoPositionConfig positionConfig;
  public static SystemState startingState = NothingAuto.NOTHING;

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
          // positionConfig.AllianceZoneSide(),
          positionConfig.TowerShootingPose(),
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
            positionConfig.TowerShootingPose(), 
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

  public enum NZLiteAuto implements SystemState {
    START {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.AcrossBumpNZPosition(),
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
            positionConfig.AcrossBumpNZPosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return OVERBUMP;
        
        return this;
      }
    },
    OVERBUMP {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.NeutralZoneStartPosition(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(2),
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
            positionConfig.NeutralZoneStartPosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return PLOW;

        return this;
      }
    },
    PLOW {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.NeutralZoneEndPosition(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(2),
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
            positionConfig.NeutralZoneEndPosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return PREOVERRAMP;

        return this;
      }
    },
    PREOVERRAMP {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().stopIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.AcrossBumpNZPosition(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(2),
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
            positionConfig.AcrossBumpNZPosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return OVERRAMP;

        return this;
      }
    },
    OVERRAMP {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().stopIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.AcrossBumpAZPosition(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(2),
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
            positionConfig.AcrossBumpAZPosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return START_SHOOT;

        return this;
      }
    },
    START_SHOOT {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.TowerShootingPose(),
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
            positionConfig.TowerShootingPose(), 
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

  public enum NZMaxAuto implements SystemState {
    START {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.AcrossBumpNZPosition(),
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
            positionConfig.AcrossBumpNZPosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return OVERBUMP;
        
        return this;
      }
    },
    OVERBUMP {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.NeutralZoneStartPosition(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(2),
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
            positionConfig.NeutralZoneStartPosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return PLOW;

        return this;
      }
    },
    PLOW {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.NeutralZoneEndPositionFull(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(2),
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
            positionConfig.NeutralZoneEndPositionFull(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return OVERRAMP;

        return this;
      }
    },
    OVERRAMP {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().stopIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.AcrossBumpAZOppositePosition(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(2),
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
            positionConfig.AcrossBumpAZOppositePosition(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return START_SHOOT;

        return this;
      }
    },
    START_SHOOT {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.TowerShootingPose(),
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
            positionConfig.TowerShootingPose(), 
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

  public enum DepotAuto implements SystemState {
    START {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
      }

      @Override
      public void execute() {
         DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.DepotEnterPose(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.TowerShootingPose(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return PLOW;

        return this;
      }
    },
    PLOW {
      @Override
      public void execute() {
         DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.DepotExitPose(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED.div(3),
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.DepotExitPose(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return GO_TO_DUMB_SHOOT;

        return this;
      }
    },
    GO_TO_DUMB_SHOOT {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          // positionConfig.AllianceZoneSide(),
          positionConfig.TowerShootingPose(),
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
            positionConfig.TowerShootingPose(), 
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
      case "Basic Shoot":
        startingState = BasicShootAuto.START;
        break;
      case "Neutral Zone Lite":
        startingState = NZLiteAuto.START;
        break;
      case "Neutral Zone Max":
        startingState = NZMaxAuto.START;
        break;
      case "Depot":
        startingState = DepotAuto.START;
        break;
      case "Nothing":
      default:
        startingState = NothingAuto.NOTHING;
        break;
    }
  }

  public AutoHoncho() {
    super(startingState);
    if (startingState == null) {
      Logger.recordOutput(getName() + "/creationAutoType", "null");
    } else {
      Logger.recordOutput(getName() + "/creationAutoType", startingState.toString());
    }
  }

  public void periodic() {
    if (getState() == null) {
      Logger.recordOutput(getName() + "/currentState", "null");
    } else {
      Logger.recordOutput(getName() + "/currentState", getState().toString());
    }
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
