package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@TeleOp(name = "SensorBot: Rotate 90 degrees", group = "feelz")
public class SensorBotTurnyThing extends SensorBotTemplate {

    private Timer async;
    private Orientation angles;
    private boolean prevA;
    private long beginTime;

    //--------------------------------------------------------------------------
    //
    //
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public SensorBotTurnyThing() {
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

        async = new Timer();

        beginTime = 0;
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
    @Override public void loop () {
        //double mtrPwr = System.currentTimeMillis() * Math.PI / 1250.0;

        //setLeftPow(-Math.sin(mtrPwr));
        //setRightPow(Math.sin(mtrPwr));

        //telemetry.addData("Rotation (Yaw,Roll,Pitch)", navx.getYaw() + ", " + navx.getRoll() + ", " + navx.getPitch());
        //telemetry.addData("Gyroscope (XYZ)", navx.getRawGyroX() + ", " + navx.getRawGyroY() + ", " + navx.getRawGyroZ());
        //telemetry.addData("Accelerometer (XYZ)", navx.getRawAccelX() + ", " + navx.getRawAccelY() + ", " + navx.getRawAccelZ());
        //telemetry.addData("Quaternion (WXYZ)", navx.getQuaternionW() + ", " + navx.getQuaternionX() + ", " + navx.getQuaternionY() + ", " + navx.getQuaternionZ());
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ);

        if (beginTime == 0) {
            beginTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - beginTime >= 5000)
            if (!gamepad1.a && prevA)
                async.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        double currOrientation = 0;
                        double origOrientation = toDegrees(angles.angleUnit, angles.thirdAngle);
                        setLeftPow(0.9);
                        setRightPow(-0.9);
                        while (Math.abs((currOrientation = toDegrees(angles.angleUnit, angles.thirdAngle)) - origOrientation) <= 180) {
                            telemetry.addData("Angle", formatDegrees(currOrientation));
                        }
                        setLeftPow(0);
                        setRightPow(0);
                    }
                }, 0);

        prevA = gamepad1.a;
    }

    double toDegrees(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(toDegrees(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

} // PootisBotManual
