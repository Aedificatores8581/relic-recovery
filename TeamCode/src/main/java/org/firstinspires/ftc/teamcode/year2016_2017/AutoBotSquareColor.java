package org.firstinspires.ftc.teamcode.year2016_2017;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.teamcode.Constants;

import for_camera_opmodes.OpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "Square Up w/ Color", group = "Autonomous")
@Disabled
public class AutoBotSquareColor extends OpModeCamera {


// Touch sensor is "ts1".
    DcMotorController mc1;
    DcMotor left, right;
    UltrasonicSensor uss1, uss2;

    DcMotor bll, blr;
    TouchSensor ts1, ts2;

    final double powah = -0.165;

    long timeBeforeFive;

    long timeDuringSquare;

    long timeAfterAlign = 0;

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

        resetEncoders();

        left.setDirection(Constants.L_MTR_DIR_2016);
        right.setDirection(Constants.R_MTR_DIR_2016);

        timeDuringSquare = 0;

        color = -1;

        timeAfterAlign = 0;

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
        for (int x = 31; x <= 40; x++) {
            for (int y = rgbImage.getHeight() / 6; y < rgbImage.getHeight() * 5 / 6; y++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        return highestColor(redValue, 0, blueValue);
    }

    public enum BeaconState {
        BLUE_RIGHT,
        BLUE_LEFT,
        BLUE,
        RED;
    }

    private BeaconState getBeaconState() {
        while (!imageReady());
        int redValue, blueValue;
        double leftRatio, rightRatio;
        Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, 2);

        // Bottom Left
        redValue = 0;
        blueValue = 0;
        for (int y = rgbImage.getHeight() / 2; y < rgbImage.getHeight(); y++) {
            for (int x = 22; x < 40; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        telemetry.addData("Blue Value Left", blueValue);
        telemetry.addData("Red Value Left", redValue);
        leftRatio = (double)blueValue / (double)redValue;
        telemetry.addData("Left Ratio", leftRatio);

        // Bottom Right
        redValue = 0;
        blueValue = 0;
        for (int y = 0; y < rgbImage.getHeight() / 2; y++) {
            for (int x = 22; x < 40; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        telemetry.addData("Blue Value Right", blueValue);
        telemetry.addData("Red Value Right", redValue);
        rightRatio = (double)blueValue / (double)redValue;
        telemetry.addData("Right Ratio", rightRatio);

        if (leftRatio > Constants.BLUE_THRESHOLD_2016 && rightRatio > Constants.BLUE_THRESHOLD_2016)
            return BeaconState.BLUE;
        else if (leftRatio > Constants.BLUE_THRESHOLD_2016 && rightRatio <= Constants.BLUE_THRESHOLD_2016)
            return BeaconState.BLUE_LEFT;
        else if (leftRatio <= Constants.BLUE_THRESHOLD_2016 && rightRatio > Constants.BLUE_THRESHOLD_2016)
            return BeaconState.BLUE_RIGHT;
        else if (leftRatio <= Constants.BLUE_THRESHOLD_2016 && rightRatio <= Constants.BLUE_THRESHOLD_2016)
            return BeaconState.RED;
        return null;
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

    public boolean isValidUssValue(double val) {
        return val >= 10 && val <= 250;
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

        BeaconState bs = getBeaconState();

        String status;
        double v1 = uss1.getUltrasonicLevel(), v2 = uss2.getUltrasonicLevel();

        if (System.currentTimeMillis() - timeAfterAlign >= 250l && isValidUssValue(v1) && isValidUssValue(v2)) {
            boolean blueOnRight = bs != BeaconState.BLUE_LEFT;
            double diff2 = v2 - v1;
            if ((diff2 >= 1.0 && diff2 <= 3.0 && blueOnRight) || (diff2 <= -1.0 && diff2 >= -3.0 && !blueOnRight)) {
                left.setPower(0);
                right.setPower(0);
                status = "Aligned correctly.";
                timeAfterAlign = System.currentTimeMillis();
            } else if (blueOnRight) {
                if (diff2 < 1.0) {
                    left.setPower(-0.25);
                    right.setPower(0.25);
                    status = "Turning left...";
                } else {
                    left.setPower(0.25);
                    right.setPower(-0.25);
                    status = "Turning right...";
                }
            } else {
                if (diff2 > -1.0) {
                    left.setPower(0.25);
                    right.setPower(-0.25);
                    status = "Turning right...";
                } else {
                    left.setPower(-0.25);
                    right.setPower(0.25);
                    status = "Turning left...";
                }
            }
        }
        else
            status = "Aligned correctly";

        /*
        telemetry.addData("Left Motor Pos.", getLeftPos());
        telemetry.addData("Right Motor Pos.", getRightPos());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());
        telemetry.addData("USS Right Value", getAverage(rightUssValues));
        telemetry.addData("USS Left Value", getAverage(leftUssValues));
        */

        telemetry.addData("Left Value", uss1.getUltrasonicLevel());
        telemetry.addData("Right Value", uss2.getUltrasonicLevel());

        telemetry.addData("Beacon State", bs);
        if (status == null)
            telemetry.addData("ERROR", "Invalid status!");
        else
            telemetry.addData("Align Status", status);

        /*
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);
        */
        telemetry.addData("Elapsed Time", (System.currentTimeMillis() - beginningTime) / 1000l);

    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}