package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameHelpers;

public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  
  private static LEDSubsystem s_ledSubsystem;
  private CANdle m_candle;
  private LEDFillHandler m_shiftFillHandler;
  private LEDFillHandler m_disabledFillHandler;
  private RainbowAnimation m_rainbowAnim;

  private boolean m_wasAutoEnabled = false;

  private LEDSubsystem() {
    m_candle = new CANdle(Constants.LED.CANDLE_ID);
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
    Logger.recordOutput(getName() + "wasAutoEnabled", m_wasAutoEnabled);

    m_disabledFillHandler.setColor(
      DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ?
        Constants.LED.BLUE_COLOR :
        Constants.LED.RED_COLOR
    );

    if (GameHelpers.isHubActive()) {
      m_shiftFillHandler.setColor(Constants.LED.ACTIVE_COLOR);
    } else {
      m_shiftFillHandler.setColor(Constants.LED.INACTIVE_COLOR);
    }
    
    if (DriverStation.isDisabled()) {
      Logger.recordOutput(getName() + "/status", "Disabled");
      m_candle.clearAnimation(0);
      m_disabledFillHandler.updateLeds(1);
    } else if (DriverStation.isAutonomousEnabled()) {
      Logger.recordOutput(getName() + "/status", "Auto Enabled");
      if (!m_wasAutoEnabled) {
        m_candle.animate(
          m_rainbowAnim
        );
      }
    } else if (DriverStation.isTeleopEnabled()) {
      m_candle.clearAnimation(0);
      Logger.recordOutput(getName() + "/status", "Teleop Enabled");
      double scoringTimeLeft = GameHelpers.scoringTimeLeft();
      if (scoringTimeLeft >= 0) {
        // our shift
        // scoringTimeLeft is between 0 and 55
        double percent = scoringTimeLeft / GameHelpers.MAX_SHIFT_TIME;
        m_shiftFillHandler.updateLeds(percent);
      } else {
        // not our shift - gamehelpers returns negative
        // so we know scoringTimeLeft is between 0 and 25
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
