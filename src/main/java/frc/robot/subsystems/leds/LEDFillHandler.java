package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.MathUtil;

public class LEDFillHandler {
  private CANdle m_candle;
  private int m_startIndex;
  private int m_endIndex;
  private boolean increasing;
  private RGBWColor m_color;

  public LEDFillHandler(
    CANdle candle,
    int startIndex,
    int endIndex,
    boolean increasing,
    RGBWColor color
  ) {
    m_candle = candle;
    m_startIndex = startIndex;
    m_endIndex = endIndex;
    m_color = color;
  }

  public void setColor(RGBWColor newColor) {
    m_color = newColor;
  }

  public void updateLeds(double percent) {
    percent = MathUtil.clamp(percent, 0, 1);
    int ledCount = m_endIndex - m_startIndex;
    int lightUpCount = (int)(ledCount * percent);
    int darkCount = ledCount - lightUpCount;
    if (increasing) {
      // low to high
      // so 50% is top half
      // set the light up count with reference to end index
      // set the rest to dark
      // The indices are inclusive
      m_candle.setLEDs(
        m_color.Red,
        m_color.Green,
        m_color.Blue,
        m_color.White,
        m_endIndex - lightUpCount,
        lightUpCount
      );
      m_candle.setLEDs(
        0,
        0,
        0,
        0,
        m_startIndex,
        darkCount
      );
    } else {
      // high to low
      // 50% is bottom half
      // so set the first light up count to on
      // and set the rest to black
      m_candle.setLEDs(
        m_color.Red,
        m_color.Green,
        m_color.Blue,
        m_color.White,
        m_startIndex,
        lightUpCount
      );
      m_candle.setLEDs(
        0,
        0,
        0,
        0,
        m_endIndex - darkCount,
        darkCount
      );
    }
  }
}