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

@Autonomous(name = "Autonomous Squaring", group = "Autonomous")
@Disabled
public class AutoBotSquareUp extends OpModeCamera {


    // Touch sensor is "ts1".
    DcMotorController mc1;
    DcMotor left, right;
    UltrasonicSensor uss1, uss2;
    Servo s1, s2;

    DcMotor bll, blr;
    TouchSensor ts1, ts2;

    final double powah = -0.165;

    boolean firstLoop;

    public static final double DIFF_TO_TICKS = 620.0 / 34.0;

    long beginningTime;

    int state;

    double diff;

    public static final double LEFT_SPEED_FACTOR = 1.27 * (67.0 / 51.0);
    public static final double TICKS_FROM_INCHES = 3700.0 / 70.0;
    public static final int RIGHT_TARGET_DISTANCE = (int) (TICKS_FROM_INCHES * 55.0 * Math.PI / 2.0) - 400;
    public static final int LEFT_TARGET_DISTANCE = (int) (RIGHT_TARGET_DISTANCE / LEFT_SPEED_FACTOR);

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

        state = 0;

        firstLoop = true;

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

    public boolean isValidUssValue(UltrasonicSensor uss) {
        return uss.getUltrasonicLevel() < 250 && uss.getUltrasonicLevel() > 7.5;
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
        String status;
        diff = getReading(uss2) - getReading(uss1);

        if (Math.abs(diff) <= 2.0) {
            left.setPower(0);
            right.setPower(0);
            status = "Squared up!";
        }
        else if (diff > 0) {
            left.setPower(0.2);
            right.setPower(-0.2);
            status = "Turning left.";
        }
        else {
            left.setPower(-0.2);
            right.setPower(0.2);
            status = "Turning right.";
        }

        /*
        telemetry.addData("Left Motor Pos.", getLeftPos());
        telemetry.addData("Right Motor Pos.", getRightPos());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());
        */

        telemetry.addData("USS Right Value", uss2.getUltrasonicLevel());
        telemetry.addData("USS Left Value", uss1.getUltrasonicLevel());

        telemetry.addData("Difference", diff);

        /*
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);
        */

    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}