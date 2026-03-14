package frc.robot.subsystems.leds;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameHelpers;

public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  
  private static LEDSubsystem s_ledSubsystem;
  private CANdle m_candle;
  private LEDFillHandler m_fillHandlerLeft;
  private LEDFillHandler m_fillHandlerRight;

  private LEDSubsystem() {
    m_candle = new CANdle(Constants.LED.CANDLE_ID);
    m_fillHandlerLeft = new LEDFillHandler(
      m_candle,
      Constants.LED.START_INDEX_LEFT,
      Constants.LED.END_INDEX_LEFT,
      true,
      Constants.LED.DEFAULT_COLOR
    );
    m_fillHandlerRight = new LEDFillHandler(
      m_candle,
      Constants.LED.START_INDEX_RIGHT,
      Constants.LED.END_INDEX_RIGHT,
      true,
      Constants.LED.DEFAULT_COLOR
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
    if (GameHelpers.isHubActive()) {
      m_fillHandlerLeft.setColor(Constants.LED.ACTIVE_COLOR);
      m_fillHandlerRight.setColor(Constants.LED.ACTIVE_COLOR);
    } else {
      m_fillHandlerLeft.setColor(Constants.LED.INACTIVE_COLOR);
      m_fillHandlerRight.setColor(Constants.LED.INACTIVE_COLOR);
    }

    double scoringTimeLeft = GameHelpers.scoringTimeLeft();
    if (scoringTimeLeft >= 0) {
      // our shift
      // scoringTimeLeft is between 0 and 55
      double percent = scoringTimeLeft / GameHelpers.MAX_SHIFT_TIME;
      m_fillHandlerLeft.updateLeds(percent);
      m_fillHandlerRight.updateLeds(percent);
    } else {
      // not our shift - gamehelpers returns negative
      // so we know scoringTimeLeft is between 0 and 25
      // TODO maybe add a check for scoringTime < -25?
      double trueTimeLeft = scoringTimeLeft + 25;
      double percent = trueTimeLeft / GameHelpers.MAX_SHIFT_TIME;
      m_fillHandlerLeft.updateLeds(percent);
      m_fillHandlerRight.updateLeds(percent);
    }
  }

  @Override
  public void close() {
    m_candle.close();
  }
}
