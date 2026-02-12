package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase implements AutoCloseable {

  private static HopperSubsystem s_hopperSubsystem;
  private CANrange m_topRange1;
  private CANrange m_topRange2;
  private CANrange m_topRange3;
  private CANrange m_bottomRange;

  private boolean m_topPrevious;
  private boolean m_bottomPrevious;
  private Timer m_topTimer;
  private Timer m_bottomTimer;

  public static HopperSubsystem getInstance() {
    if (s_hopperSubsystem == null) {
      s_hopperSubsystem = new HopperSubsystem();
    }
    return s_hopperSubsystem;
  }

  private HopperSubsystem() {
    m_topRange1 = new CANrange(Constants.Hopper.CANRANGE_TOP_ONE_ID);
    m_topRange2 = new CANrange(Constants.Hopper.CANRANGE_TOP_TWO_ID);
    m_topRange3 = new CANrange(Constants.Hopper.CANRANGE_TOP_THREE_ID);
    m_bottomRange = new CANrange(Constants.Hopper.CANRANGE_BOTTOM_ID);

    m_topTimer = new Timer();
    m_bottomTimer = new Timer();
  }

  /**
   * Periodic function to refresh the class variables of HopperSubsystem.
   * Should only be called from {@link frc.robot.Robot#robotPeriodic() Robot.java}.
   */
  @Override
  public void periodic() {
    // Check the top row as a collective
    boolean topCurrent = (
      // TODO: test if this is OR or AND
      canRangeBlocked(m_topRange1) ||
      canRangeBlocked(m_topRange2) ||
      canRangeBlocked(m_topRange3)
    );
    // Only one canrange on the bottom
    boolean bottomCurrent = (
      canRangeBlocked(m_bottomRange)
    );

    // This logic applies for both top and bottom:
    // If it's blocked in real life but the variable
    // doesn't have it as blocked, consider that
    // a rising edge & start the timer
    // If the opposite is true (not blocked in real
    // life and the variable does have it as blocked),
    // stop & reset the timer
    if (topCurrent && !m_topPrevious) {
      m_topTimer.start();
    }
    if (!topCurrent && m_topPrevious) {
      m_topTimer.stop();
      m_topTimer.reset();
    }

    if (bottomCurrent && !m_bottomPrevious) {
      m_bottomTimer.start();
    }
    if (!bottomCurrent && m_bottomPrevious) {
      m_bottomTimer.stop();
      m_bottomTimer.reset();
    }

    m_topPrevious = topCurrent;
    m_bottomPrevious = bottomCurrent;
  }

  /**
   * Checks that at least one of the top beam breaks has been blocked for
   * {@link frc.robot.Constants.Hopper#DELAY_TIME a certain amount of time}.
   * @return true if a beambreak is blocked and that the timer has elapsed
   * the desired amount of time, false otherwise
   */
  public boolean getTopRow() {
    return m_topPrevious &&
      m_topTimer.hasElapsed(Constants.Hopper.DELAY_TIME);
  }

  /**
   * Checks that at least one of the bottom beam breaks has been blocked for
   * {@link frc.robot.Constants.Hopper#DELAY_TIME a certain amount of time}.
   * @return true if a beambreak is blocked and that the timer has elapsed
   * the desired amount of time, false otherwise
   */
  public boolean getBottomRow() {
    return m_bottomPrevious &&
      m_bottomTimer.hasElapsed(Constants.Hopper.DELAY_TIME);
  }

  /**
   * Gets the distance measured by a CANrange and compares it to
   * {@link frc.robot.Constants.Hopper#BLOCKED_DISTANCE a certain distance}
   * @param range The CANrange that the distance should be checked for
   * @return true if the measured value is less than or equal to the
   * constant distance
   */
  private boolean canRangeBlocked(CANrange range) {
    return range.getDistance().getValue().lte(
      Constants.Hopper.BLOCKED_DISTANCE
    );
  }

  @Override
  public void close() {
    m_bottomRange.close();
    m_topRange1.close();
    m_topRange2.close();
    m_topRange3.close();
  }
}
