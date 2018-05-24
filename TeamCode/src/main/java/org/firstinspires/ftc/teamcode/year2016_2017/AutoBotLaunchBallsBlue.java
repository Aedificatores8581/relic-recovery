package org.firstinspires.ftc.teamcode.year2016_2017;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants;

import for_camera_opmodes.OpModeCamera;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "[BLUE] LAUNCH BALLS AND RAMP", group = "Autonomous")
@Disabled
public class AutoBotLaunchBallsBlue extends OpModeCamera {


// Touch sensor is "ts1".
    DcMotorController mc1;
    DcMotor left, right;
    State state = State.STATE_FIRST;
    UltrasonicSensor uss1, uss2;
    Servo s1, s2;
	VoltageSensor vs;

    DcMotor bll, blr;
    TouchSensor ts1, ts2;

    final double powah = -0.2;

    double ballLaunchSpeed;

    boolean blueOnRight;

    long timeBeforeFive;

    long timeDuringSquare;

    long timeBeforeFive2b;

    long beginningTime;
    int color;

    public static final double LEFT_SPEED_FACTOR = 1.35 * (67.0 / 51.0);
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
		vs = hardwareMap.voltageSensor.get("mc1");

        resetEncoders();

        left.setDirection(Constants.L_MTR_DIR_2016);
        right.setDirection(Constants.R_MTR_DIR_2016);

        state = State.STATE_FIRST;

        timeDuringSquare = 0;

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
        for (int x = 31; x <= 40; x++) {
            for (int y = 0; y < rgbImage.getHeight(); y++) {
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
        int redValue, blueValue;
        double leftRatio, rightRatio;
        Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, 2);

        // Bottom Left
        redValue = 0;
        blueValue = 0;
        for (int y = 0; y < rgbImage.getHeight() / 2; y++) {
            for (int x = 31; x <= 40; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        leftRatio = blueValue / redValue;

        // Bottom Right
        redValue = 0;
        blueValue = 0;
        for (int y = rgbImage.getHeight() / 2; y < rgbImage.getHeight(); y++) {
            for (int x = 31; x <= 40; x++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
            }
        }
        rightRatio = blueValue / redValue;

        if (leftRatio > Constants.BLUE_THRESHOLD_2016 && rightRatio > Constants.BLUE_THRESHOLD_2016)
            return BeaconState.BLUE;
        else if (leftRatio > Constants.BLUE_THRESHOLD_2016)
            return BeaconState.BLUE_RIGHT; // This is weird.
        else if (rightRatio > Constants.BLUE_THRESHOLD_2016)
            return BeaconState.BLUE_LEFT; // Very weird.
        else
            return BeaconState.RED;
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

    public enum State {
        STATE_FIRST,
        STATE_WAIT,
        STATE_DELAY,
        STATE_FORWARDS_BEFORE_ARC,
        STATE_LAUNCH_BALLS,
        STATE_TURN_TO_RAMP,
        STATE_DRIVE_TO_RAMP,
        STATE_FINALLY_DONE_YAY
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

        switch (state) {
            case STATE_FIRST: // Initialization
                beginningTime = System.currentTimeMillis(); // Used for elapsed time in telemetry.
                state = State.STATE_WAIT;
            case STATE_WAIT:
                if (System.currentTimeMillis() - beginningTime >= Constants.WAIT_TIME_2016) {
                    s1.setPosition(Constants.SERVO_MIN_2016); // Lower launch servo to bottom.
                    s2.setPosition(0.33); // Do something to the gate.
                    left.setPower(-powah * 0.625);
                    right.setPower(-powah * 0.625);
                    ballLaunchSpeed = 0.28 * Constants.BALL_LAUNCHER_DIR_2016;
                    timeBeforeFive = System.currentTimeMillis();
                    state = State.STATE_DELAY;
                }
                break;
            case STATE_DELAY:
                if (System.currentTimeMillis() - timeBeforeFive >= 5000) {
                    bll.setPower(-ballLaunchSpeed);
                    blr.setPower(-ballLaunchSpeed);
                    state = State.STATE_FORWARDS_BEFORE_ARC;
                }
                break;
            case STATE_FORWARDS_BEFORE_ARC: // Going forwards.
                if ((getLeftPos() <= -585 * Constants.ENCODER_MULT * 1.2) && (getRightPos() <= -585 * Constants.ENCODER_MULT * 1.2)) {
                    resetEncoders();
                    left.setPower(0);
                    right.setPower(0);
                    state = State.STATE_LAUNCH_BALLS;
                }
                break;
            case STATE_LAUNCH_BALLS: // Launch balls.
                if (System.currentTimeMillis() - timeBeforeFive >= 10000) { // Reverse chronological order.
                    bll.setPower(0);
                    blr.setPower(0);
                    s1.setPosition(Constants.SERVO_MIN_2016); // Drop launch servo.
                    s2.setPosition(1.0); // Raise gate.
                    left.setPower(-powah * 1.1);
                    right.setPower(powah * 1.1);
                    state = State.STATE_TURN_TO_RAMP;
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 9000) {
                    s1.setPosition(Constants.SERVO_MAX_2016); // Launch second ball.
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 8000) {
                    s2.setPosition(0.1); // Lower gate.
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 7000) {
                    s1.setPosition(Constants.SERVO_MIN_2016); // Drop launch servo.
                    bll.setPower(-(ballLaunchSpeed - 0.01));
                    blr.setPower(-(ballLaunchSpeed - 0.01));
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 6000) {
                    s1.setPosition(Constants.SERVO_MAX_2016); // Launch first ball.
                }
                break;
            case STATE_TURN_TO_RAMP:
                if (getLeftPos() <= -290 * 1.2 && getRightPos() >= 290 * 1.2) {
                    resetEncoders();
                    left.setPower(-powah);
                    right.setPower(-powah);
                    state = State.STATE_DRIVE_TO_RAMP;
                }
                break;
            case STATE_DRIVE_TO_RAMP:
                if (getLeftPos() <= -2600 * 1.2 && getRightPos() <= -2600 * 1.2) {
                    left.setPower(0.0);
                    right.setPower(0.0);
                    state = State.STATE_FINALLY_DONE_YAY;
                }
                break;
            case STATE_FINALLY_DONE_YAY:
                break;
        }

        /*
        telemetry.addData("Left Motor Pos.", getLeftPos());
        telemetry.addData("Right Motor Pos.", getRightPos());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());
        */
        telemetry.addData("Raw Right Value", uss2.getUltrasonicLevel());
        telemetry.addData("Raw Left Value", uss1.getUltrasonicLevel());
        /*
        telemetry.addData("USS Right Value", getAverage(rightUssValues));
        telemetry.addData("USS Left Value", getAverage(leftUssValues));
        */
		telemetry.addData("Voltage", vs.getVoltage());
		
        telemetry.addData("State", state);

        if (color != -1)
            telemetry.addData("Color", colorName(color));

        /*
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);
        */
        telemetry.addData("Elapsed Time", (System.currentTimeMillis() - beginningTime) / 1000l);

        telemetry.addData("Image Width", width);
        telemetry.addData("Image Height", height);

        telemetry.addData("Blue on Right", blueOnRight);

    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}