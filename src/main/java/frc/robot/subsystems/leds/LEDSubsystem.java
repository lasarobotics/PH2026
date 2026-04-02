package frc.robot.subsystems.leds;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameHelpers;

// phoenix 5 is deprecated
// whatev
@SuppressWarnings("removal")
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  
  private static LEDSubsystem s_ledSubsystem;
  private CANdle m_candle;
  private LEDFillHandler m_shiftFillHandler;
  private LEDFillHandler m_disabledFillHandler;
  private RainbowAnimation m_rainbowAnim;

  private boolean m_wasAutoEnabled = false;

  private LEDSubsystem() {
    m_candle = new CANdle(Constants.LED.CANDLE_ID, "canivore");
    m_shiftFillHandler = new LEDFillHandler(
      m_candle,
      Constants.LED.START_INDEX,
      Constants.LED.END_INDEX,
      true,
      Constants.LED.DEFAULT_COLOR
    );
    m_disabledFillHandler = new LEDFillHandler(
      m_candle,
      Constants.LED.START_INDEX,
      Constants.LED.END_INDEX,
      true,
      Constants.LED.DEFAULT_COLOR
    );
    
    m_rainbowAnim = new RainbowAnimation(
      1,
      0.8,
      Constants.LED.END_INDEX - Constants.LED.START_INDEX,
      true,
      Constants.LED.START_INDEX
    );
  }

  public static LEDSubsystem getInstance() {
    if (s_ledSubsystem == null) {
      s_ledSubsystem = new LEDSubsystem();
    }
    return s_ledSubsystem;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDSubsystem/wasAutoEnabled", m_wasAutoEnabled);

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      m_disabledFillHandler.setColor(Constants.LED.WHITE_COLOR);
    } else {
      m_disabledFillHandler.setColor(
        alliance.get().equals(Alliance.Blue) ?
          Constants.LED.BLUE_COLOR :
          Constants.LED.RED_COLOR
      );
    }
    
    if (DriverStation.isDisabled()) {
      // disabled
      Logger.recordOutput("LEDSubsystem/status", "Disabled");
      m_candle.clearAnimation(0);
      m_disabledFillHandler.updateLeds(1);
    } else if (DriverStation.isAutonomousEnabled()) {
      // enabled in auto
      Logger.recordOutput("LEDSubsystem/status", "Auto Enabled");
      if (!m_wasAutoEnabled) {
        m_candle.animate(
          m_rainbowAnim
        );
      }
    } else if (DriverStation.isTeleopEnabled()) {
      // enabled in teleop
      m_candle.clearAnimation(0);
      Logger.recordOutput("LEDSubsystem/status", "Teleop Enabled");
      double scoringTimeLeft = GameHelpers.scoringTimeLeft();
      if (scoringTimeLeft == GameHelpers.DEFAULT_SCORING_TIME_LEFT) {
        // No gamedata/something has gone wrong with getting the scoring time left
        // show warning color
        m_shiftFillHandler.setColor(Constants.LED.WARNING_COLOR);
        m_shiftFillHandler.updateLeds(1);
      } else if (scoringTimeLeft >= 0) {
        // our shift
      // scoringTimeLeft is between 0 and 55`1
        boolean even = ((int)scoringTimeLeft) % 2 == 0;
        if (
          GameHelpers.scoringTimeLeft() < 5 &&
          even
        ) {
          m_shiftFillHandler.setColor(Constants.LED.WHITE_COLOR);
        } else {
          m_shiftFillHandler.setColor(Constants.LED.ACTIVE_COLOR);
        }
        double percent = scoringTimeLeft / GameHelpers.MAX_SHIFT_TIME;
        m_shiftFillHandler.updateLeds(percent);
      } else {
        // not our shift - gamehelpers returns negative
        // so we know scoringTimeLeft is between 0 and 25
        m_shiftFillHandler.setColor(Constants.LED.INACTIVE_COLOR);
        double trueTimeLeft = scoringTimeLeft + 25;
        double percent = trueTimeLeft / GameHelpers.MAX_SHIFT_TIME;
        m_shiftFillHandler.updateLeds(percent);
      }
    }

    m_wasAutoEnabled = DriverStation.isAutonomousEnabled();
  }

  @Override
  public void close() {
    m_candle.destroyObject();
  }
}
