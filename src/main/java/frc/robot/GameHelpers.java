package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameHelpers {

    private static double lastMatchTime = 21; // auto time + 1
    private static Timer oneSecondTimer = new Timer();

    /**
     * Check match time and reset the oneSecondTimer
     * if it's different from the last time it was checked.
     * Should only be called in
     * {@link frc.robot.Robot#robotPeriodic() robotPeriodic()}.
     * This method *should* always stay synced to the
     * current match time pretty closely because
     * it's run at the start of robotPeriodic.
     */
    public static void periodicTimerUpdater() {
        double time = DriverStation.getMatchTime();

        // just rolled over
        if (time != lastMatchTime) {
            oneSecondTimer.reset();
            oneSecondTimer.start();
        }

        lastMatchTime = time;

        Logger.recordOutput("GameHelpers/oneSecondTimer", oneSecondTimer.get());
        Logger.recordOutput("GameHelpers/lastMatchTime", lastMatchTime);
    }

    /**
     * Returns the match time left in the current period
     * (auto or teleop). Basically a more precise version
     * of the builtin
     * {@link edu.wpi.first.wpilibj.DriverStation#getMatchTime() getMatchTime()}.
     * This is because the FMS counts down in seconds, which
     * isn't precise enough to make the way that we're using it this
     * year useful.
     * @return The time left in the match (in seconds)
     */
    public static double matchTimeLeft() {
        return lastMatchTime - oneSecondTimer.get();
    }

    /**
     * Helper method that checks game specific message &
     * alliances to see if current alliance won auto.
     * @return 1 if current alliance won auto, 0 if
     * current alliance lost, -1 if error
     */
    public static int wonAuto() {
        String gameData = DriverStation.getGameSpecificMessage();
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

        boolean wonAuto = (autoAlliance == 'B' && currentAlliance.get().equals(Alliance.Blue)) ||
                          (autoAlliance == 'R' && currentAlliance.get().equals(Alliance.Red));
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
            return (!s1 && !s3);
        } else {
            boolean s2 = 80 <= time && time <= 105;
            boolean s4 = 30 <= time && time <= 55;
            return (!s2 && !s4);
        }
    }

    final static double[][] wonAutoActiveIntervals = new double[][] { {140, 130}, {105, 80}, {55, 0} }; // won auto (s2, s4)
    final static double[][] lostAutoActiveIntervals = new double[][] { {140, 105}, {80, 55}, {30, 0} }; // lost auto (s1, s3)

    /**
     * This method gets the amount of time until the current alliance's
     * hub becomes inactive. If the hub is inactive, returns a negative
     * number representing how long it has been since the last active
     * period ended.
     * @return The amount of time left to score (in seconds)
     */
    public static double scoringTimeLeft() {
        double time = matchTimeLeft();

        if (DriverStation.isAutonomous()) {
            return Double.MAX_VALUE;
        }
        if (time <= 0) {
            return 0;
        }

        int wonNumber = wonAuto();
        if (wonNumber == -1) {
            return Double.MAX_VALUE;
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
            if (i == 0) continue; // don't want any index out of bounds errors
            // the thinking is this:
            // imagine the a & bs (etc) as the starts and ends
            // of active periods and the X as where we are
            // a...b...X...c...d...e...f
            // so in the case that the check for a..b has already failed
            // and we're checking c..d, we can check the previous interval (a..b)
            // to see if we're less than its lower bound
            // and if we are (i.e. we're past the previous interval)
            // we subtract that from the game time to get ✨ negative time ✨
            double prevEnd = activeIntervals[i - 1][1];
            // the interval[0] check is to make sure that we're actually
            // in the range between this one and the previous
            if (interval[0] <= time && time <= prevEnd) {
                return time - prevEnd;
            }
        }

        // I'm not sure how we would actually get here, especially
        // accounting for the negative logic in the loop
        return Double.MAX_VALUE;
    }
}
