package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoPositionConfig.Quadrant;
import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoHoncho extends StateMachine implements AutoCloseable {
  public enum NothingAuto implements SystemState {
    NOTHING {
      @Override
      public void initialize() {
        s_wantToCrossBump = false;
        s_wantToDumbShoot = false;
        s_wantToShoot = false;
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    }
  }

  public enum BasicShootAuto implements SystemState {
    SHOOT {
      Timer stopShootTimer = new Timer();

      @Override
      public void initialize() {
        DriveSubsystem.getInstance().driveAutoAim();
        stopShootTimer.reset();
        stopShootTimer.start();
        s_wantToShoot = true;
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToShoot = false;
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        // don't want to waste battery
        if (stopShootTimer.hasElapsed(Constants.Auto.EightBallShootingTime)) {
          return END;
        }

        return this;
      }
    },
    END {
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
        if (positionConfig == null) {
          return;
        }

        DriveSubsystem.getInstance().goTo(
          positionConfig.AcrossBumpNZ(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (
          !DriverStation.isAutonomous() ||
          positionConfig == null
        ) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AcrossBumpNZ(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return TO_PLOW_START;
        
        return this;
      }
    },
    TO_PLOW_START {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.NeutralZoneStart(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.NeutralZoneStart(), 
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
          positionConfig.NeutralZoneEnd(),
          MetersPerSecond.of(1),
          Constants.Drive.MAX_SPEED.div(2),
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public void end(boolean interrupted) {
        // we want it out but not running lol
        IntakeSubsystem.getInstance().stopIntake();
        IntakeSubsystem.getInstance().deployIntake();
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.NeutralZoneEnd(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return PREOVERRAMP;

        return this;
      }
    },
    PREOVERRAMP {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AcrossBumpNZ(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AcrossBumpNZ(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE,
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return OVERRAMP;

        return this;
      }
    },
    OVERRAMP {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AcrossBumpAZ(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AcrossBumpAZ(), 
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
        DriveSubsystem.getInstance().stopMoving();
        DriveSubsystem.getInstance().driveAutoAim();
        s_wantToShoot = true;
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToShoot = false;
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
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
      }

      @Override
      public void execute() {
        if (positionConfig == null) {
          return;
        }

        DriveSubsystem.getInstance().goTo(
          positionConfig.AcrossBumpNZ(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (
          !DriverStation.isAutonomous() ||
          positionConfig == null
        ) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AcrossBumpNZ(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return TO_PLOW_START;
        
        return this;
      }
    },
    TO_PLOW_START {
      @Override
      public void initialize() {
        IntakeSubsystem.getInstance().startIntake();
      }

      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.NeutralZoneStart(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.NeutralZoneStart(), 
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
          positionConfig.NeutralZoneEndFull(),
          MetersPerSecond.of(1),
          Constants.Drive.MAX_SPEED.div(2),
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public void end(boolean interrupted) {
        // we want it out but not running lol
        IntakeSubsystem.getInstance().stopIntake();
        IntakeSubsystem.getInstance().deployIntake();
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.NeutralZoneEndFull(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE, 
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return OVERRAMP;

        return this;
      }
    },
    PREOVERRAMP {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AcrossBumpNZOpposite(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AcrossBumpNZOpposite(), 
            Constants.Auto.HIGH_DISTANCE_TOLERANCE,
            Constants.Auto.HIGH_ROTATION_TOLERANCE
          )
        ) return OVERRAMP;

        return this;
      }
    },
    OVERRAMP {
      @Override
      public void execute() {
        DriveSubsystem.getInstance().goTo(
          positionConfig.AcrossBumpAZOpposite(),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (!DriverStation.isAutonomous()) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.AcrossBumpAZOpposite(), 
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
        DriveSubsystem.getInstance().stopMoving();
        DriveSubsystem.getInstance().driveAutoAim();
        s_wantToShoot = true;
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToShoot = false;
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
        if (positionConfig == null) {
          return;
        }

        DriveSubsystem.getInstance().goTo(
          positionConfig.DepotEnterPose(),
          MetersPerSecond.of(0),
          Constants.Drive.MAX_SPEED,
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public SystemState nextState() {
        if (
          !DriverStation.isAutonomous() ||
          positionConfig == null
        ) return NothingAuto.NOTHING;

        if (
          DriveSubsystem.atDestination(
            positionConfig.DepotEnterPose(), 
            Constants.Auto.LOW_DISTANCE_TOLERANCE, 
            Constants.Auto.LOW_ROTATION_TOLERANCE
          )
        ) return PLOW;

        return this;
      }
    },
    PLOW {
      @Override
      public void execute() {
         DriveSubsystem.getInstance().goTo(
          positionConfig.DepotExitPose(),
          MetersPerSecond.of(1),
          Constants.Drive.MAX_SPEED.div(3),
          Constants.Drive.MAX_ANGULAR_RATE
        );
      }

      @Override
      public void end(boolean interrupted) {
        // we want it out but not running lol
        IntakeSubsystem.getInstance().stopIntake();
        IntakeSubsystem.getInstance().deployIntake();
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
        ) return GO_TO_SHOOT;

        return this;
      }
    },
    GO_TO_SHOOT {
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
        DriveSubsystem.getInstance().stopMoving();
        DriveSubsystem.getInstance().driveAutoAim();
        s_wantToShoot = true;
      }

      @Override
      public void end(boolean interrupted) {
        s_wantToShoot = false;
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

  public static AutoPositionConfig positionConfig;
  public static SystemState startingState = NothingAuto.NOTHING;

  public static void setAutoQuadrant(String quadrant) {
    Logger.recordOutput("AutoHoncho/setAutoQuadrant", quadrant);
    switch (quadrant) {
      case "Blue Left":
        positionConfig = new AutoPositionConfig(Quadrant.BLUE_LEFT);
        break;
      case "Blue Right":
        positionConfig = new AutoPositionConfig(Quadrant.BLUE_RIGHT);
        break;
      case "Red Left":
        positionConfig = new AutoPositionConfig(Quadrant.RED_LEFT);
        break;
      case "Red Right":
        positionConfig = new AutoPositionConfig(Quadrant.RED_RIGHT);
        break;
      default:
        break;
    }
  }

  public static void setAutoType(String type) {
    Logger.recordOutput("AutoHoncho/setAutoType", type);
    switch (type) {
      case "Basic Shoot":
        startingState = BasicShootAuto.SHOOT;
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
        // avoid null pointer exception
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
