package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// TODO find a better way to do this
class DriverStationException extends Exception {}

public class Helpers {
    public static boolean wonAuto() throws DriverStationException {
        String gameData = DriverStation.getGameSpecificMessage();
        // should never happen
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
        double time = DriverStation.getMatchTime();
        // 140 seconds in match
        // there are less inactive periods than active ones overall, so we check those
        // Shift timings:
        // S1: 130-105
        // S2: 105-80
        // S3: 80-55
        // S4: 55-30
        if (wonAuto) {
            boolean s1 = time <= 130 && time >= 105;
            boolean s3 = time <= 80 && time >= 55;
            return (!s1 && !s3);
        } else {
            boolean s2 = time <= 105 && time >= 80;
            boolean s4 = time <= 55 && time >= 30;
            return (!s2 && !s4);
        }
    }

    /**
     * This method gets the amount of time until the current alliance's
     * hub becomes inactive. If the hub is inactive, returns 0.
     * @return The amount of time left to score (in seconds)
     */
    public static double scoringTimeLeft() {
        double time = DriverStation.getMatchTime();

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
