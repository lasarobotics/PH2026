package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Random;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Most of this code is a carbon copy of the logic in
 * {@link org.littletonrobotics.junction.wpilog.WPILOGWriter WPILOGWriter}
 */
public class LoggingInitializer extends StateMachine {

  public enum LoggingStates implements SystemState {
    START {
      @Override
      public void execute() {
        if (
          (
            DriverStation.isDSAttached() &&
            HAL.getSystemTimeValid()
          ) ||
          RobotBase.isSimulation()
        ) {
          if (dsAttachedTime == null) {
            dsAttachedTime = RobotController.getFPGATime() / 1000000.0;
          }
          
          // Set logDate if we've waited long enough OR if we are in simulation
          if (logDate == null) {
            if (RobotBase.isSimulation() || 
                (RobotController.getFPGATime() / 1000000.0 - dsAttachedTime > timestampUpdateDelay)) {
              logDate = LocalDateTime.now();
            }
          }
        } else {
          dsAttachedTime = null;
        }

        MatchType matchType = DriverStation.getMatchType();
        if (logMatchText == null && matchType != MatchType.None) {
          logMatchText = "";
          switch (matchType) {
            case Practice:
              logMatchText = "p";
              break;
            case Qualification:
              logMatchText = "q";
              break;
            case Elimination:
              logMatchText = "e";
              break;
            default:
              break;
          }
          logMatchText += Integer.toString(DriverStation.getMatchNumber());
        }

        // Update folder
        StringBuilder folderNameBuilder = new StringBuilder();
        folderNameBuilder.append("418_");
        // something like 01-01-01 or abc123
        if (logDate == null) {
          folderNameBuilder.append(randomIdentifier);
        } else {
          folderNameBuilder.append(timeFormatter.format(logDate));
        }
        String eventName = DriverStation.getEventName().toLowerCase();
        // if event, abc123_txbel
        if (eventName.length() > 0) {
          folderNameBuilder.append("_");
          folderNameBuilder.append(eventName);
        }
        // if match type/number, abc123_txbel_q44
        if (logMatchText != null) {
          folderNameBuilder.append("_");
          folderNameBuilder.append(logMatchText);
        }
        folder = basePath.resolve(folderNameBuilder.toString());
      }

      @Override
      public SystemState nextState() {
        // there's an extra 0.5 second delay on top of the 5 seconds
        // so the robot has time to get things like match type/number/time/etc
        if (
          RobotBase.isSimulation() ||
          (
            dsAttachedTime != null &&
            RobotController.getFPGATime() / 1000000.0 - dsAttachedTime
              > timestampUpdateDelay + 0.5
          )
        ) return LOGGING;

        return this;
      }
    },
    LOGGING {
      @Override
      public void initialize() {
        try {
          Files.createDirectories(folder);
        } catch (IOException e) {
          // probably won't happen
          e.printStackTrace();
        }
        
        // advantagekit
        Logger.addDataReceiver(new WPILOGWriter(
          folder.resolve("akit_log.wpilog").toString()
        ));
        Logger.addDataReceiver(new NT4Publisher());

        Logger.start();

        Logger.recordMetadata("ProjectName", "PH2026");
        Logger.recordMetadata("RuntimeType", Robot.isSimulation() ? "sim" : "real");

        // ctre
        SignalLogger.setPath(folder.toString());
        SignalLogger.start();
      }

      @Override
      public SystemState nextState() {
        // basically if it's a real match and teleop is over
        // it's probably the end of the match
        if (shouldStopLogging) {
          return END;
        }

        return this;
      }
    },
    END {
      @Override
      public void initialize() {
        if (DriverStation.getMatchType() != MatchType.None) {
          // 165 is the max
          // 140 sec teleop + 3 sec delay + 20 sec auto = 163
          // so might as well get all of it
          LimelightHelpers.triggerRewindCapture(Constants.Drive.SHOOTER_LIMELIGHT_NAME, 165);
          LimelightHelpers.triggerRewindCapture(Constants.Drive.RIGHT_LIMELIGHT_NAME, 165);
        }
      }

      @Override
      public SystemState nextState() {
        return this;
      }
    }
  }
  
  private static LoggingInitializer s_loggingInitializer;

  private static final double timestampUpdateDelay =
    2.5; // Wait several seconds after DS attached to ensure
         // timestamp/timezone is updated
  private static final Path defaultPathRio = Path.of("/U/logs");
  private static final Path defaultPathSim = Path.of("logs");
  private static final DateTimeFormatter timeFormatter =
      DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");

  private static Path basePath;
  private static Path folder;
  private static String randomIdentifier;
  private static Double dsAttachedTime;

  private static LocalDateTime logDate;
  private static String logMatchText;

  // private static LoggedNetworkBoolean shouldStopLogging = new LoggedNetworkBoolean("/Tuning/LoggingInitializer/shouldStopLogging");
  private static boolean shouldStopLogging = false;

  public static LoggingInitializer getInstance() {
    if (s_loggingInitializer == null) {
      s_loggingInitializer = new LoggingInitializer();
    }
    return s_loggingInitializer;
  }

  public LoggingInitializer() {
    super(LoggingStates.START);
    basePath = RobotBase.isSimulation() ? defaultPathSim : defaultPathRio;
    folder = basePath; // avoid nullpointerexception

    Random random = new Random();
    StringBuilder randomIdentifierBuilder = new StringBuilder();
    for (int i = 0; i < 4; i++) {
      randomIdentifierBuilder.append(String.format("%04x", random.nextInt(0x10000)));
    }
    randomIdentifier = randomIdentifierBuilder.toString();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/state", getState().toString());
    // Logger.recordOutput(getName() + "/test", shouldStopLogging.get());
  }

  public void stopLogging() {
    shouldStopLogging = true;
  }
}
