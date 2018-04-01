package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Conjured into existence by The Saminator on 11-24-2017.
 */
public class GyroAngles {
    public static final AxesOrder ORDER = AxesOrder.ZYX;
    public static final AngleUnit UNIT = AngleUnit.DEGREES;

    private final double z, y, x;

    public GyroAngles(Orientation angles) {
        z = AngleUnit.DEGREES.fromUnit(UNIT, angles.firstAngle);
        y = AngleUnit.DEGREES.fromUnit(UNIT, angles.secondAngle);
        x = AngleUnit.DEGREES.fromUnit(UNIT, angles.thirdAngle);
    }

    public double getZ() {
        return z;
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

    public static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
