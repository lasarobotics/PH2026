package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class DriverStationException extends Exception {}

public class GameHelpers {

    private static double lastMatchTime = 21; // auto time + 1
    private static Timer oneSecondTimer = new Timer();

    /**
     * Check match time and reset the oneSecondTimer
     * if it's different from the last time it was checked.
     * Should only be called in
     * {@link frc.robot.Robot#robotPeriodic() robotPeriodic()}.
     */
    public static void periodicTimerUpdater() {
        double time = DriverStation.getMatchTime();

        // just rolled over
        if (time != lastMatchTime) {
            oneSecondTimer.reset();
            oneSecondTimer.start();
        }

        lastMatchTime = time;
    }

    /**
     * Returns the match time left in the current period
     * (auto or teleop). Basically a more precise version
     * of the builtin
     * {@link edu.wpi.first.wpilibj.Timer#getMatchTime() getMatchTime()}.
     * This is because the FMS counts down in seconds, which
     * isn't precise enough to make the way that we're using it this
     * year useful.
     * @return
     */
    public static double matchTimeLeft() {
        return lastMatchTime - oneSecondTimer.get();
    }

    /**
     * Helper method that checks game specific message &
     * alliances to see if current alliance won auto.
     * @return True if the current alliance won auto.
     * @throws DriverStationException if no alliance is available,
     * or if gamedata is not available
     */
    public static boolean wonAuto() throws DriverStationException {
        String gameData = DriverStation.getGameSpecificMessage();
        // probably happens during auto
        if (gameData.length() != 1) {
            throw new DriverStationException();
        }
        char autoAlliance = gameData.charAt(0);

        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        // should never happen
        if (currentAlliance.isEmpty()) {
            throw new DriverStationException();
        }

        boolean wonAuto = (autoAlliance == 'B' && currentAlliance.get().equals(Alliance.Blue)) ||
                          (autoAlliance == 'R' && currentAlliance.get().equals(Alliance.Red));
        return wonAuto;
    }

    /**
     * Checks game data and match time to see if the hub is active
     * @return If the hub is active for the current alliance
     */
    public static boolean isHubActive() {
        if (DriverStation.isAutonomous()) {
            return true;
        }

        boolean wonAuto;
        try {
            wonAuto = wonAuto();
        } catch (DriverStationException e) {
            // assume true
            return true;
        }

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

    /**
     * This method gets the amount of time until the current alliance's
     * hub becomes inactive. If the hub is inactive, returns 0.
     * @return The amount of time left to score (in seconds)
     */
    public static double scoringTimeLeft() {
        double time = matchTimeLeft();

        if (DriverStation.isAutonomous()) {
            return Double.POSITIVE_INFINITY;
        }
        if (time <= 0) {
            return 0;
        }

        boolean wonAuto;
        try {
            wonAuto = wonAuto();
        } catch (DriverStationException e) {
            // shouldn't happen, just say infinity
            return Double.POSITIVE_INFINITY;
        }

        // inactive intervals (start, end) where start > end and time is counting down
        double[][] inactiveIntervals = wonAuto
            ? new double[][] { {130, 105}, {80, 55} }
            : new double[][] { {105, 80}, {55, 30} };

        // if currently in an inactive interval, return 0
        for (double[] interval : inactiveIntervals) {
            double start = interval[0];
            double end = interval[1];
            if (time <= start && time >= end) {
                return 0;
            }
        }

        // find the next upcoming inactive interval (time > start), return time until it begins
        double minUntil = Double.POSITIVE_INFINITY;
        for (double[] interval : inactiveIntervals) {
            double start = interval[0];
            if (time > start) {
                double until = time - start;
                if (until < minUntil) {
                    minUntil = until;
                }
            }
        }

        return minUntil;
    }
}
