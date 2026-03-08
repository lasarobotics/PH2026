package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Random;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Most of this code is a carbon copy of the logic in
 * {@link org.littletonrobotics.junction.wpilog.WPILOGWriter WPILOGWriter}
 */
public class LoggingInitializer extends Command {
  private static final double timestampUpdateDelay =
    5.0; // Wait several seconds after DS attached to ensure
         // timestamp/timezone is updated
  private static final Path defaultPathRio = Path.of("/U/logs");
  private static final Path defaultPathSim = Path.of("logs");
  private static final DateTimeFormatter timeFormatter =
      DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");

  private Path basePath;
  private Path folder;
  private final String randomIdentifier;
  private Double dsAttachedTime;

  private LocalDateTime logDate;
  private String logMatchText;

  public LoggingInitializer() {
    basePath = RobotBase.isSimulation() ? defaultPathSim : defaultPathRio;
    folder = basePath; // Initialize to avoid NPE if end() is called before execute()

    Random random = new Random();
    StringBuilder randomIdentifierBuilder = new StringBuilder();
    for (int i = 0; i < 4; i++) {
      randomIdentifierBuilder.append(String.format("%04x", random.nextInt(0x10000)));
    }
    randomIdentifier = randomIdentifierBuilder.toString();
  }

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
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    try {
      Files.createDirectories(folder);
    } catch (IOException e) {
      // probably won't happen
      // TODO figure out what to do here (if anything)
      e.printStackTrace();
    }
    
    // advantagekit
    Logger.addDataReceiver(new WPILOGWriter(
      folder.resolve("akit_log.wpilog").toString()
    ));
    Logger.addDataReceiver(new NT4Publisher());

    Logger.start();

    // ctre
    SignalLogger.setPath(folder.toString());
    SignalLogger.start();
  }

  @Override
  public boolean isFinished() {
    // there's an extra 0.5 second delay on top of the 5 seconds
    // so the robot has time to get things like match type/number/time/etc
    return (
      RobotBase.isSimulation() ||
      (
        dsAttachedTime != null &&
        RobotController.getFPGATime() / 1000000.0 - dsAttachedTime
          > timestampUpdateDelay + 0.5
      )
    );
  }
}
