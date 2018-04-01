package org.firstinspires.ftc.teamcode;

/**
 * Conjured into existence by The Saminator on 02-27-2018.
 */
public class Utilities {
    public static double clamp(double min, double test, double max) {
        if (max < min) {
            double temp = min;
            min = max;
            max = temp;
        }
        return Math.max(Math.min(max, test), min);
    }

    public static boolean withinTolerance(double test, double limit, double tolerance) {
        return test == clamp(limit - tolerance, test, limit + tolerance);
    }
}
