package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@Autonomous(name = "SensorBot: Telemetry", group = "feelz")
public class SensorBotTelemetry extends SensorBotTemplate {

    Orientation angles;
    Acceleration gravity;

    //--------------------------------------------------------------------------
    //
    //
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public SensorBotTelemetry() {
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
        telemetry.addData("(COLOR) Red", color.red());
        telemetry.addData("(COLOR) Green", color.green());
        telemetry.addData("(COLOR) Blue", color.blue());
        telemetry.addData("(COLOR) Total", color.alpha());
        //telemetry.addData("(ODS) Light Detected", ods.getLightDetected());
        //telemetry.addData("(TOUCH) Is touched?", touch.isPressed());
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ);
        gravity = imu.getGravity();

        telemetry.addData("(NAV) Status", imu.getSystemStatus().toShortString());
        telemetry.addData("(NAV) Calib.", imu.getCalibrationStatus());
        telemetry.addData("(NAV) Pitch", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("(NAV) Roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("(NAV) Yaw", formatAngle(angles.angleUnit, angles.thirdAngle));
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

} // PootisBotManual