package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@TeleOp(name = "RevBot: Tele-Op", group = "bepis")

public class SmolBotManual extends SmolBotTemplate
{

    //--------------------------------------------------------------------------
    //
    //
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
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


    public SmolBotManual() {
        //
        // Initialize base classes.
        //
        // All via self-construction.
        //
        // Initialize class members.
        //
        // All via self-construction.

    }

    double armpower = 0.8;
    double handpower = 0.8;
    @Override
    public void start() {
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
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double green = colors.green;
        double blue = colors.blue;
        double red = colors.red;

        setLeftPow(left);
        setRightPow(right);
        /**while (gamepad1.a)
            while (gamepad1.x)
                if (green/(blue + red + green) < .38)
                    setLeftPow(1);
                    setRightPow(1);
                if (green/(blue + red + green) > .4)
                    setLeftPow (0);
                    setRightPow(1);
            while (gamepad1.b)
                if (green/(blue + red + green) < .38)
                    setLeftPow(1);
                    setRightPow(1);
                if (green/(blue + red + green) > .4)
                    setLeftPow (1);
                    setRightPow(0);
**/
        if (gamepad1.left_stick_button)
            setGrabPow(1);
        else if (gamepad1.right_stick_button)
            setGrabPow(-1);
        else
            setGrabPow(0);

        if (gamepad1.right_bumper)
            setArmPow(armpower);
        else if (gamepad1.right_trigger > 0)
            setArmPow(-armpower);
        else
            setArmPow(0);

        if (gamepad1.left_bumper)
            setHandPow(handpower);
        else if (gamepad1.left_trigger > 0)
            setHandPow(-handpower);
        else
            setHandPow(0);
        if (colors.green/(colors.blue + colors.red + colors.green) > .4)
            setLeftPow(-1);
            setRightPow(1);






        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angle   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addData("Left Pow", left);
        telemetry.addData("Right Pow", right);
        colors = colorSensor.getNormalizedColors();

        telemetry.addLine()
                .addData("a", colors.alpha )
                .addData("r", (colors.red/(colors.blue + colors.red + colors.green)))
                .addData("g", (colors.green/(colors.blue + colors.red + colors.green)))
                .addData("b", (colors.blue/(colors.blue + colors.red + colors.green)));

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                });

        telemetry.addLine()
                .addData("angle", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angle.angleUnit, angle.firstAngle);
                    }
                });


        telemetry.addLine()
                .addData("gravity", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
        telemetry.update();
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "", AngleUnit.DEGREES.normalize(degrees));
    }



        /*
         * white red about .36111
         * white green about .34722
         * white blue about .29166
         */


}

 // PootisBotManual
