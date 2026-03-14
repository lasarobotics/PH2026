package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class GameHelpers {

  private static Timer timer = new Timer();

  public static void zeroTimer() {
    timer.reset();
    timer.start();
  }

  /**
   * Returns the match time left in the current period
   * (only works for teleop)
   * @return The time left in the match (in seconds)
   */
  public static double matchTimeLeft() {
    return 140 - timer.get();
  }

  /**
   * Helper method that checks game specific message &
   * alliances to see if current alliance won auto.
   * @return 1 if current alliance won auto, 0 if
   * current alliance lost, -1 if error
   */
  public static int wonAuto() {
    String gameData = DriverStation.getGameSpecificMessage();
    Logger.recordOutput("GameHelpers/gameData", gameData);
    // probably happens during auto
    if (gameData.length() != 1) {
      return -1;
    }
    char autoAlliance = gameData.charAt(0);

    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    // should never happen
    if (currentAlliance.isEmpty()) {
      return -1;
    }

    boolean wonAuto = (
      (autoAlliance == 'B' && currentAlliance.get().equals(Alliance.Blue)) ||
      (autoAlliance == 'R' && currentAlliance.get().equals(Alliance.Red))
    );
    return wonAuto ? 1 : 0;
  }

  /**
   * Checks game data and match time to see if the hub is active
   * @return If the hub is active for the current alliance
   */
  public static boolean isHubActive() {
    if (DriverStation.isAutonomous()) {
      return true;
    }

    int wonNumber = wonAuto();
    if (wonNumber == -1) {
      return true;
    }
    boolean wonAuto = wonNumber == 1;

    // if in practice mode/real field, always counts down
    double time = matchTimeLeft();
    // 140 seconds in match
    // there are less inactive periods than active ones overall, so we check those
    // Shift timings:
    // S1: 130-105
    // S2: 105-80
    // S3: 80-55
    // S4: 55-30
    if (wonAuto) {
      boolean s1 = 105 <= time && time <= 130;
      boolean s3 = 55 <= time && time <= 80;
      // not in either inactive period
      return (!(s1 || s3));
    } else {
      boolean s2 = 80 <= time && time <= 105;
      boolean s4 = 30 <= time && time <= 55;
      // not in either inactive period
      return (!(s2 || s4));
    }
  }

  public static final double[][] wonAutoActiveIntervals =
    new double[][] { {140, 130}, {105, 80}, {55, 0} }; // won auto (s2, s4)
  public static final double[][] lostAutoActiveIntervals = 
    new double[][] { {140, 105}, {80, 55}, {30, 0} }; // lost auto (s1, s3)
  // 55 to 0 if won auto
  // this is for LEDs - we want a certain number of LEDs on to always
  // represent the same amount of clock time
  public static final double MAX_SHIFT_TIME = 55;

  /**
   * This method gets the amount of time until the current alliance's
   * hub becomes inactive. If the hub is inactive, returns a negative
   * number representing how long it has been since the last active
   * period ended. If there is an invalid state (e.g. auton, no game data)
   * then return 50. This is a big enough number for any usage but small
   * enough to make logging it in AdvantageScope usable.
   * @return The amount of time left to score (in seconds)
   */
  public static double scoringTimeLeft() {
    double time = matchTimeLeft();

    if (DriverStation.isAutonomous()) {
      return 50;
    }
    if (time <= 0) {
      return 0;
    }

    int wonNumber = wonAuto();
    if (wonNumber == -1) {
      return 50;
    }
    boolean wonAuto = (wonNumber == 1);

    // start > end, timer counts down
    double[][] activeIntervals = wonAuto
      ? wonAutoActiveIntervals
      : lostAutoActiveIntervals;

    // it's just like they taught us in ap!
    for (int i = 0; i < activeIntervals.length; i++) {
      double[] interval = activeIntervals[i];
      // interval[1] always less than interval[0]
      if (interval[1] <= time && time <= interval[0]) {
        return time - interval[1];
      }
      // at this point, we know we aren't in the current interval
      // but what if we went past the previous?
      // the thinking is this:
      // imagine the a & bs (etc) as the starts and ends
      // of active periods and the X as where we are
      // a...b...X...c...d...e...f
      // so in the case that the check for a..b has already failed
      // and we're checking c..d, we can check the previous interval (a..b)
      // to see if we're less than its lower bound
      // and if we are (i.e. we're past the previous interval)
      // we subtract that from the game time to get ✨ negative time ✨
      if (i != 0) { // don't want any index out of bounds errors
        double prevEnd = activeIntervals[i - 1][1];
        // the interval[0] check is to make sure that we're actually
        // in the range between this one and the previous
        if (interval[0] <= time && time <= prevEnd) {
          return time - prevEnd;
        }
      }
    }

    // I'm not sure how we would actually get here, especially
    // accounting for the negative logic in the loop
    System.out.println("WTF?");
    return 50;
  }
}
