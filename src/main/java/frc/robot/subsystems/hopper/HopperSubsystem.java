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
  private CANrange m_bottomRange1;
  private CANrange m_bottomRange2;
  private CANrange m_bottomRange3;

  private boolean m_topBlocked;
  private boolean m_bottomBlocked;
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
    m_bottomRange1 = new CANrange(Constants.Hopper.CANRANGE_BOTTOM_ONE_ID);
    m_bottomRange2 = new CANrange(Constants.Hopper.CANRANGE_BOTTOM_TWO_ID);
    m_bottomRange3 = new CANrange(Constants.Hopper.CANRANGE_BOTTOM_THREE_ID);

    m_topTimer = new Timer();
    m_bottomTimer = new Timer();
  }

  /**
   * Periodic function to refresh the class variables of HopperSubsystem.
   * Should only be called from {@link frc.robot.Robot#robotPeriodic Robot.java}.
   */
  public void hopperPeriodic() {
    boolean topB = (
      canRangeBlocked(m_topRange1) ||
      canRangeBlocked(m_topRange2) ||
      canRangeBlocked(m_topRange3)
    );
    boolean bottomB = (
      canRangeBlocked(m_bottomRange1) ||
      canRangeBlocked(m_bottomRange2) ||
      canRangeBlocked(m_bottomRange3)
    );

    if (topB && !m_topBlocked) {
      m_topTimer.start();
    }
    if (!topB && m_topBlocked) {
      m_topTimer.stop();
      m_topTimer.reset();
    }

    if (bottomB && !m_bottomBlocked) {
      m_bottomTimer.start();
    }
    if (!bottomB && m_bottomBlocked) {
      m_bottomTimer.stop();
      m_bottomTimer.reset();
    }

    m_topBlocked = topB;
    m_bottomBlocked = bottomB;
  }

  // TODO javadoc
  public boolean getTopRow() {
    return m_topBlocked &&
      m_topTimer.hasElapsed(Constants.Hopper.DELAY_TIME);
  }

  // TODO javadoc
  public boolean getBottomRow() {
    return m_bottomBlocked &&
      m_bottomTimer.hasElapsed(Constants.Hopper.DELAY_TIME);
  }

  // TODO maybe use different values for top and bottom?
  // TODO write javadoc
  private boolean canRangeBlocked(CANrange range) {
    return range.getDistance().getValue().lte(
      Constants.Hopper.BLOCKED_DISTANCE
    );
  }

  @Override
  public void close() {
    m_bottomRange1.close();
    m_bottomRange2.close();
    m_bottomRange3.close();
    m_topRange1.close();
    m_topRange2.close();
    m_topRange3.close();
  }
}
