package org.firstinspires.ftc.teamcode.year2016_2017;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.Constants;

import for_camera_opmodes.OpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "Autonomous Color", group = "Autonomous")
@Disabled
public class AutoBotColor extends OpModeCamera {


// Touch sensor is "ts1".
    DcMotorController mc1;
    DcMotor left, right;
    UltrasonicSensor uss1, uss2;
    Servo s1, s2;

    DcMotor bll, blr;
    TouchSensor ts1, ts2;

    final double powah = -0.165;

    long timeBeforeFive;

    long beginningTime;
    int color;

    public static final double LEFT_SPEED_FACTOR = 1.27 * (67.0 / 51.0);
    public static final double TICKS_FROM_INCHES = 3700.0 / 70.0;
    public static final int RIGHT_TARGET_DISTANCE = (int)(TICKS_FROM_INCHES * 55.0 * Math.PI / 2.0) - 400;
    public static final int LEFT_TARGET_DISTANCE = (int)(RIGHT_TARGET_DISTANCE / LEFT_SPEED_FACTOR);

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
    */
    @Override
    public void init() {
        setCameraDownsampling(8);
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        mc1 = hardwareMap.dcMotorController.get("mc1");
        uss1 = hardwareMap.ultrasonicSensor.get("uss1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");
        ts1 = hardwareMap.touchSensor.get("ts1");
        ts2 = hardwareMap.touchSensor.get("ts2");
        bll = hardwareMap.dcMotor.get("bll");
        blr = hardwareMap.dcMotor.get("blr");
        s1 = hardwareMap.servo.get("s1"); // Launch servo
        s2 = hardwareMap.servo.get("s2"); // Gate servo

        resetEncoders();

        left.setDirection(Constants.L_MTR_DIR_2016);
        right.setDirection(Constants.R_MTR_DIR_2016);
        color = -1;

        super.init(); // inits camera functions, starts preview callback
    }

    public String colorName(int colorID) {
        switch (colorID) {
            case 0:
                return "RED";
            case 1:
                return "GREEN";
            case 2:
                return "BLUE";
            default:
                return "URINE YELLOW";
        }
    }

    private int getColor() {
        int redValue = 0, blueValue = 0;
        Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, 2);
        for (int x = rgbImage.getWidth() / 3; x < rgbImage.getWidth() * 2 / 3; x++) {
            for (int y = rgbImage.getHeight() / 6; y < rgbImage.getHeight() * 5 / 6; y++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        return highestColor(redValue, 0, blueValue);
    }

    public void resetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getLeftPos() {
        return left.getCurrentPosition() * Constants.ENCODER_DIR_2016;
    }

    public int getRightPos() {
        return right.getCurrentPosition() * Constants.ENCODER_DIR_2016;
    }

    public double getReading(UltrasonicSensor uss) {
        double value;
        do {
            value = uss.getUltrasonicLevel();
        } while (value >= 250 || value <= 7.5);
        return value;
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

        int redValue = 0, blueValue = 0;
        int rgbW, rgbH;
        long systemTime = System.currentTimeMillis();

        Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);
        rgbW = rgbImage.getWidth();
        rgbH = rgbImage.getHeight();
        /*for (int x = 0; x < rgbImage.getWidth(); x += 6) {
            redValue = 0;
            blueValue = 0;
            for (int y = 0; y < rgbImage.getHeight(); y++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
            telemetry.addData("Blue", blueValue);
            telemetry.addData("Red", redValue);
            telemetry.addData("Width", x);
        }
        for (int y = 0; y < rgbImage.getHeight(); y += 6) {
            redValue = 0;
            blueValue = 0;
            for (int x = 42; x <= 66; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
            telemetry.addData("Blue", blueValue);
            telemetry.addData("Red", redValue);
            telemetry.addData("Height", y);
        }*/

        redValue = 0;
        blueValue = 0;
        for (int y = 0; y < rgbImage.getHeight() / 2; y++) {
            for (int x = 42; x <= 60; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        telemetry.addData("Top Left", true);
        telemetry.addData("Blue", blueValue);
        telemetry.addData("Red", redValue);

        redValue = 0;
        blueValue = 0;
        for (int y = rgbImage.getHeight() / 2; y < rgbImage.getHeight(); y++) {
            for (int x = 42; x <= 60; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        telemetry.addData("Top Right", true);
        telemetry.addData("Blue", blueValue);
        telemetry.addData("Red", redValue);

        redValue = 0;
        blueValue = 0;
        for (int y = 0; y < rgbImage.getHeight() / 2; y++) {
            for (int x = 62; x <= 80; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        telemetry.addData("Bottom Left", true);
        telemetry.addData("Blue", blueValue);
        telemetry.addData("Red", redValue);

        redValue = 0;
        blueValue = 0;
        for (int y = rgbImage.getHeight() / 2; y < rgbImage.getHeight(); y++) {
            for (int x = 62; x <= 80; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        telemetry.addData("Bottom Right", true);
        telemetry.addData("Blue", blueValue);
        telemetry.addData("Red", redValue);

        /*
        telemetry.addData("Left Motor Pos.", getLeftPos());
        telemetry.addData("Right Motor Pos.", getRightPos());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());
        */
        /*
        telemetry.addData("USS Right Value", getAverage(rightUssValues));
        telemetry.addData("USS Left Value", getAverage(leftUssValues));
        */
        telemetry.addData("Red", redValue);
        telemetry.addData("Blue", blueValue);
        /*
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);
        */
        telemetry.addData("USS R", uss2.getUltrasonicLevel());
        telemetry.addData("USS L", uss1.getUltrasonicLevel());

        telemetry.addData("Image Width", width);
        telemetry.addData("Image Height", height);

        telemetry.addData("Current Time", System.currentTimeMillis() - systemTime);

        telemetry.addData("RGB Width", rgbW);
        telemetry.addData("RGB Height", rgbH);

    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}