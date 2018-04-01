package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * Created by fgpor on 3/3/2018.
 */

@Disabled
@Autonomous(name = "Balance", group = "no")
public class DriveBotBalance extends DriveBotTestTemplate {
    static class Angles {
        public static final double PHI_BASELINE = -130;
        public static final double PHI_TOLERANCE = 10;
        public static final double THETA_BASELINE = 0;
        public static final double THETA_TOLERANCE = 7.5;

        public static boolean withinPhiLimits(double phi) {
            return Utilities.withinTolerance(phi, Angles.PHI_BASELINE, Angles.PHI_TOLERANCE);
        }

        public static boolean withinThetaLimits(double theta) {
            return Utilities.withinTolerance(theta, Angles.THETA_BASELINE, Angles.THETA_TOLERANCE);
        }
    }

    Acceleration gravity;
    //SensorBotBalance.BalancingState balancingState;
    Spherical3D angles;


    enum BalancingState {
        BALANCING_PHI,
        BALANCING_THETA,
        BALANCING_SUCCESS
    }/*

    private void testAngles() {
        if (!SensorBotBalance.Angles.withinThetaLimits(angles.theta))
            balancingState = SensorBotBalance.BalancingState.BALANCING_THETA;
        else if (!SensorBotBalance.Angles.withinPhiLimits(angles.phi))
            balancingState = SensorBotBalance.BalancingState.BALANCING_PHI;
        else
            balancingState = SensorBotBalance.BalancingState.BALANCING_SUCCESS;
    }

    //--------------------------------------------------------------------------
    //
    //
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------*/
    public DriveBotBalance() {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.
    }

    @Override
    public void init() {
        msStuckDetectInit = 300000;

        super.init();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.getAll(BNO055IMU.class).get(0);
        imu.initialize(parameters);

        //balancingState = SensorBotBalance.BalancingState.BALANCING_SUCCESS;
    }

    //--------------------------------------------------------------------------
    //
    // loop
    //
    //-------
    // Initializes the class.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override
    public void loop() {
        gravity = imu.getGravity();
        angles = cartesianToSpherical(new Cartesian3D(gravity.xAccel, gravity.yAccel, gravity.zAccel));

        telemetry.addData("(NAV) Status", imu.getSystemStatus().toShortString());
        telemetry.addData("(NAV) Calib.", imu.getCalibrationStatus());

        telemetry.addData("Theta angle ( acos (z / sqrt(x^2 + y^2 + z^2)) )", angles.theta);
        telemetry.addData("Phi angle ( atan (y / x) )", angles.phi);
        telemetry.addData("X gravity", gravity.xAccel);
        telemetry.addData("Y gravity", gravity.yAccel);
        telemetry.addData("Z gravity", gravity.zAccel);
/*
        switch (balancingState) {
            case BALANCING_THETA:
                if (angles.theta < SensorBotBalance.Angles.THETA_BASELINE) {
                    setLeftPow(-0.125);
                    setRightPow(0.125);
                } else {
                    setLeftPow(0.125);
                    setRightPow(-0.125);
                }
                break;
            case BALANCING_PHI:
                if (angles.phi < SensorBotBalance.Angles.PHI_BASELINE) {
                    setLeftPow(-0.125);
                    setRightPow(-0.125);
                } else {
                    setLeftPow(0.125);
                    setRightPow(0.125);
                }
                break;
            case BALANCING_SUCCESS:
                setLeftPow(0.0);
                setRightPow(0.0);
                break;
        }
        testAngles();

        telemetry.addData("Balancing State", balancingState.name());*/
    }

    public Spherical3D cartesianToSpherical(Cartesian3D cartesian) {
        double
                x2 = cartesian.x * cartesian.x,
                y2 = cartesian.y * cartesian.y,
                z2 = cartesian.z * cartesian.z;
        double r = Math.sqrt(x2 + y2 + z2);
        double theta = Math.acos(cartesian.z / r) * org.firstinspires.ftc.teamcode.Constants.RADS_TO_DEGS;
        double phi = Math.atan2(cartesian.y, cartesian.x) * org.firstinspires.ftc.teamcode.Constants.RADS_TO_DEGS;
        return new Spherical3D(r, theta, phi);
    }

} // PootisBotManual

