package frc.robot.subsystems.leds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
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

  public StatusCode updateLeds(double percent) {
    percent = MathUtil.clamp(percent, 0, 1);
    int ledCount = m_endIndex - m_startIndex;
    int lightUpCount = (int)(ledCount * percent);
    return updateLeds(lightUpCount);
  }

  private StatusCode updateLeds(int lightUpCount) {
    int ledCount = m_endIndex - m_startIndex;
    int darkCount = ledCount - lightUpCount;
    if (increasing) {
      // low to high
      return m_candle.setControl(
        new SolidColor(
          m_startIndex + darkCount,
          m_endIndex
        ).withColor(
          m_color
        )
      );
    } else {
      // high to low
      return m_candle.setControl(
        new SolidColor(
          m_startIndex,
          m_endIndex - darkCount
        ).withColor(
          m_color
        )
      );
    }
  }
}