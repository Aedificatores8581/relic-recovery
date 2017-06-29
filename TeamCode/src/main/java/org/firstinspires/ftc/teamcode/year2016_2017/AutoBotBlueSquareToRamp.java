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

@Autonomous(name = "Autonomous Blue Ramp Beta", group = "Autonomous")
@Disabled
public class AutoBotBlueSquareToRamp extends OpModeCamera {


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

    long beginningTime, runningTime;
    int color;

    public static final double LEFT_SPEED_FACTOR = 1.55 * (67.0 / 51.0);
    public static final double TICKS_FROM_INCHES = 3700.0 / 70.0;
    public static final int RIGHT_TARGET_DISTANCE = (int)(TICKS_FROM_INCHES * 55.0 * Math.PI / 2.0) - 960;
    //public static final int LEFT_TARGET_DISTANCE = (int)(RIGHT_TARGET_DISTANCE / LEFT_SPEED_FACTOR);

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
        return Math.abs(left.getCurrentPosition());
    }

    public int getRightPos() {
        return Math.abs(right.getCurrentPosition());
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
        STATE_FORWARDS_BEFORE_ARC,
        STATE_GOING_ON_ARC,
        STATE_SQUARE_UP_1B,
        STATE_CHECK_SQUARING_1B,
        STATE_FWD_TO_BEACON_1B,
        STATE_BACK_UP_LAUNCH_BALL_1B,
        STATE_LAUNCH_BALLS_1B,
        STATE_CHECK_COLOR_1B,
        STATE_SET_POW_TO_DRIVE_FWD_1B,
        STATE_FWD_TO_BEACON_AGAIN_1B,
        STATE_BACK_UP_CHECK_COLOR_1B,
        STATE_BACK_UP_TURN_AWAY_1B,
        STATE_DRIVE_TO_NEXT_BEACON_2B,
        STATE_TURN_TO_FACE_BEACON_2B,
        STATE_TURN_TO_RAMP,
        STATE_GO_TO_RAMP,
        STATE_SQUARE_UP_2B,
        STATE_CHECK_SQUARING_2B,
        STATE_CHECK_TIME_2B,
        STATE_FWD_TO_BEACON_2B,
        STATE_BACK_UP_FROM_BEACON_2B,
        STATE_CHECK_COLOR_2B,
        STATE_GO_TO_HIT_BEACON_2B,
        STATE_END,
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
                s1.setPosition(Constants.SERVO_MIN_2016); // Lower launch servo to bottom.
                s2.setPosition(0.33); // Do something to the gate.

                if (vs.getVoltage() > 13.0)
                    ballLaunchSpeed = 0.28;
                else
                    ballLaunchSpeed = 0.28;

                left.setPower(powah * 2.0 / LEFT_SPEED_FACTOR);
                right.setPower(powah * 2.0);
                state = State.STATE_GOING_ON_ARC;
                break;
            case STATE_FORWARDS_BEFORE_ARC: // Going forwards.
                if ((getLeftPos() >= 50 * Constants.ENCODER_MULT) && (getRightPos() >= 50 * Constants.ENCODER_MULT)) {
                    resetEncoders();
                    left.setPower(powah * 2.0 / LEFT_SPEED_FACTOR);
                    right.setPower(powah * 2.0);
                    state = State.STATE_GOING_ON_ARC;
                }
                break;
            case STATE_GOING_ON_ARC: // Going on the long drive towards the beacon.
                //if (getLeftPos() >= LEFT_TARGET_DISTANCE * Constants.ENCODER_MULT && getRightPos() >= RIGHT_TARGET_DISTANCE * Constants.ENCODER_MULT) {
                int rightPos = getRightPos();
                if (rightPos >= RIGHT_TARGET_DISTANCE * Constants.ENCODER_MULT) {
                    resetEncoders();
                    left.setPower(0);
                    right.setPower(0);
                    state = State.STATE_SQUARE_UP_1B;
                }
                break;
            case STATE_SQUARE_UP_1B:
                double diff = uss2.getUltrasonicLevel() - uss1.getUltrasonicLevel();
                if (Math.abs(diff) <= 2.0) {
                    left.setPower(0);
                    right.setPower(0);
                    timeDuringSquare = System.currentTimeMillis();
                    state = State.STATE_CHECK_SQUARING_1B;
                }
                else if (diff > 0) {
                    left.setPower(Constants.SQUARE_POWER_2016);
                    right.setPower(-Constants.SQUARE_POWER_2016);
                }
                else {
                    left.setPower(-Constants.SQUARE_POWER_2016);
                    right.setPower(Constants.SQUARE_POWER_2016);
                }
                break;
            case STATE_CHECK_SQUARING_1B:
                if (System.currentTimeMillis() - timeDuringSquare >= 250) {
                    if (Math.abs(uss2.getUltrasonicLevel() - uss1.getUltrasonicLevel()) <= 2.0) {
                        left.setPower(powah * 0.7);
                        right.setPower(powah * 0.7);
                        state = State.STATE_FWD_TO_BEACON_1B;
                    }
                    else {
                        state = State.STATE_SQUARE_UP_1B;
                    }
                }
                break;
            case STATE_FWD_TO_BEACON_1B: // Drives forwards until it hits the button.
                if (ts1.isPressed() || ts2.isPressed()) {
                    resetEncoders();
                    left.setPower(-powah * 1.1);
                    right.setPower(-powah * 1.1);
                    bll.setPower(-ballLaunchSpeed);
                    blr.setPower(-ballLaunchSpeed);
                    state = State.STATE_BACK_UP_LAUNCH_BALL_1B;
                }
                break;
            case STATE_BACK_UP_LAUNCH_BALL_1B: // Backing up and launching ball.
                if ((getLeftPos() >= 500 * Constants.ENCODER_MULT) && (getRightPos() >= 500 * Constants.ENCODER_MULT)) {
                    left.setPower(0.0);
                    right.setPower(0.0);
                    timeBeforeFive = System.currentTimeMillis();
                    state = State.STATE_LAUNCH_BALLS_1B;
                }
                break;
            case STATE_LAUNCH_BALLS_1B: // Launch balls.
                if (System.currentTimeMillis() - timeBeforeFive >= 2700) { // Reverse chronological order.
                    bll.setPower(0);
                    blr.setPower(0);
                    s1.setPosition(Constants.SERVO_MIN_2016); // Drop launch servo.
                    s2.setPosition(1.0); // Raise gate.
                    state = State.STATE_CHECK_COLOR_1B;
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 2200) {
                    s1.setPosition(Constants.SERVO_MAX_2016); // Launch second ball.
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 1700) {
                    s2.setPosition(0.1); // Lower gate.
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 1000) {
                    s1.setPosition(Constants.SERVO_MIN_2016); // Drop launch servo.
                }
                else if (System.currentTimeMillis() - timeBeforeFive >= 600) {
                    s1.setPosition(Constants.SERVO_MAX_2016); // Launch first ball.
                }
                break;
            case STATE_CHECK_COLOR_1B: // Checks color.
                if (System.currentTimeMillis() - timeBeforeFive >= 2100) {
                    color = getColor();
                    if (color == 2) { // Target color is BLUE. Move on.
                        resetEncoders();
                        left.setPower(powah * 1.1);
                        right.setPower(-powah * 1.1);
                        state = State.STATE_BACK_UP_TURN_AWAY_1B;
                    } else {
                        state = State.STATE_SET_POW_TO_DRIVE_FWD_1B;
                    }
                }
                break;
            case STATE_SET_POW_TO_DRIVE_FWD_1B: // Set motor powers to drive forwards.
                if (System.currentTimeMillis() - timeBeforeFive >= 5750) {
                    left.setPower(powah * 0.7);
                    right.setPower(powah * 0.7);
                    state = State.STATE_FWD_TO_BEACON_AGAIN_1B;
                }
                break;
            case STATE_FWD_TO_BEACON_AGAIN_1B: // Drives forwards until it hits the button.
                if (ts1.isPressed() || ts2.isPressed()) {
                    resetEncoders();
                    left.setPower(-powah);
                    right.setPower(-powah);
                    timeBeforeFive = System.currentTimeMillis();
                    state = State.STATE_BACK_UP_CHECK_COLOR_1B;
                }
                break;
            case STATE_BACK_UP_CHECK_COLOR_1B: // Backing up and checking color.
                if ((getLeftPos() >= 500 * Constants.ENCODER_MULT) && (getRightPos() >= 500 * Constants.ENCODER_MULT)) {
                    left.setPower(0.0);
                    right.setPower(0.0);
                    state = State.STATE_CHECK_COLOR_1B;
                }
                break;
            case STATE_BACK_UP_TURN_AWAY_1B: // Turning away from the beacon.
                if (getLeftPos() >= 650 * Constants.ENCODER_MULT || getRightPos() >= 650 * Constants.ENCODER_MULT) {
                    resetEncoders();
                    left.setPower(powah * 1.5);
                    right.setPower(powah * 1.5);
                    state = State.STATE_DRIVE_TO_NEXT_BEACON_2B;
                }
                break;
            case STATE_DRIVE_TO_NEXT_BEACON_2B: // Driving towards the next beacon.
                if ((getLeftPos() >= 1575 * Constants.ENCODER_MULT) && (getRightPos() >= 1575 * Constants.ENCODER_MULT)) {
                    resetEncoders();
                    left.setPower(-powah * 1.25);
                    right.setPower(powah * 1.25);
                    state = State.STATE_TURN_TO_FACE_BEACON_2B;
                }
                break;
            case STATE_TURN_TO_FACE_BEACON_2B: // Turns towards second beacon.
                if (getLeftPos() >= 750 * Constants.ENCODER_MULT || getRightPos() >= 750 * Constants.ENCODER_MULT) {
                    left.setPower(0);
                    right.setPower(0);
                    resetEncoders();
                    state = State.STATE_SQUARE_UP_2B;
                }
                break;
            case STATE_TURN_TO_RAMP: // Turn to go hit the ball.
                if (getLeftPos() >= 490 || getRightPos() >= 490) {
                    resetEncoders();
                    left.setPower(-powah * 2);
                    right.setPower(-powah * 2);
                    state = State.STATE_GO_TO_RAMP;
                }
                break;
            case STATE_GO_TO_RAMP: // Go hit the ball.
                if (getLeftPos() >= 2800 && getRightPos() >= 2800) {
                    resetEncoders();
                    left.setPower(0);
                    right.setPower(0);
                    state = State.STATE_END;
                }
                break;
            case STATE_SQUARE_UP_2B:
                /*boolean blueOnRight = bs == BeaconState.BLUE_RIGHT;
                double v1 = uss1.getUltrasonicLevel(), v2 = uss2.getUltrasonicLevel();
                double diff2 = v2 - v1;
                if ((diff2 >= 1.0 && diff2 <= 3.0 && blueOnRight) || (diff2 <= -1.0 && diff2 >= -3.0 && !blueOnRight)) {
                    left.setPower(0);
                    right.setPower(0);
                    state = State.STATE_FWD_TO_BEACON_2B;
                } else if (blueOnRight) {
                    if (diff2 < 1.0) {
                        left.setPower(-0.25);
                        right.setPower(0.25);
                    } else {
                        left.setPower(0.25);
                        right.setPower(-0.25);
                    }
                } else {
                    if (diff2 > -1.0) {
                        left.setPower(0.25);
                        right.setPower(-0.25);
                    } else {
                        left.setPower(-0.25);
                        right.setPower(0.25);
                    }
                }*/
                double diff2 = uss2.getUltrasonicLevel() - uss1.getUltrasonicLevel();
                if (Math.abs(diff2) <= 3.0) {
                    left.setPower(0);
                    right.setPower(0);
                    timeDuringSquare = System.currentTimeMillis();
                    state = State.STATE_CHECK_SQUARING_2B;
                }
                else if (diff2 > 0) {
                    left.setPower(Constants.SQUARE_POWER_2016);
                    right.setPower(-Constants.SQUARE_POWER_2016);
                }
                else {
                    left.setPower(-Constants.SQUARE_POWER_2016);
                    right.setPower(Constants.SQUARE_POWER_2016);
                }
                break;
            case STATE_CHECK_SQUARING_2B:
                if (System.currentTimeMillis() - timeDuringSquare >= 250) {
                    if (Math.abs(uss2.getUltrasonicLevel() - uss1.getUltrasonicLevel()) <= 3.0)
                        state = State.STATE_CHECK_TIME_2B;
                    else
                        state = State.STATE_SQUARE_UP_2B;
                }
                break;
            case STATE_CHECK_TIME_2B:
                resetEncoders();
                if (System.currentTimeMillis() - beginningTime >= 22000) {
                    left.setPower(powah * 1.1);
                    right.setPower(-powah * 1.1);
                    state = State.STATE_TURN_TO_RAMP;
                }
                else {
                    left.setPower(powah * 0.7);
                    right.setPower(powah * 0.7);
                    state = State.STATE_FWD_TO_BEACON_2B;
                }
                break;
            case STATE_FWD_TO_BEACON_2B: // Hit the second beacon.
                if (ts1.isPressed() || ts2.isPressed()) {
                    resetEncoders();
                    left.setPower(-powah);
                    right.setPower(-powah);
                    state = State.STATE_BACK_UP_FROM_BEACON_2B;
                }
                break;
            case STATE_BACK_UP_FROM_BEACON_2B: // Backing up.
                if ((getLeftPos() >= 300 * Constants.ENCODER_MULT) && (getRightPos() >= 300 * Constants.ENCODER_MULT)) {
                    left.setPower(0.0);
                    right.setPower(0.0);
                    timeBeforeFive2b = System.currentTimeMillis();
                    state = State.STATE_CHECK_COLOR_2B;
                }
                break;
            case STATE_CHECK_COLOR_2B: // Check color.
                if (System.currentTimeMillis() - timeBeforeFive2b >= 2100) {
                    color = getColor();
                    if (color == 2) { // Target color is BLUE. Move on.
                        resetEncoders();
                        left.setPower(powah * 1.1);
                        right.setPower(-powah * 1.1);
                        state = State.STATE_TURN_TO_RAMP;
                    }
                    else {
                        state = State.STATE_GO_TO_HIT_BEACON_2B;
                    }
                }
                break;
            case STATE_GO_TO_HIT_BEACON_2B: // Go forwards.
                if (System.currentTimeMillis() - timeBeforeFive2b >= 5750) {
                    left.setPower(powah * 0.7);
                    right.setPower(powah * 0.7);
                    state = State.STATE_FWD_TO_BEACON_2B;
                }
                break;
        }

        /*
        telemetry.addData("Left Motor Pos.", getLeftPos());
        telemetry.addData("Right Motor Pos.", getRightPos());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());
        */

        telemetry.addData("Left Encoder", getLeftPos());
        telemetry.addData("Right Encoder", getRightPos());
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

        telemetry.addData("Ultrasonic Left", uss1.getUltrasonicLevel());
        telemetry.addData("Ultrasonic Right", uss2.getUltrasonicLevel());

        if (!state.equals(State.STATE_END))
            runningTime = System.currentTimeMillis();

        telemetry.addData("Elapsed Time", runningTime - beginningTime);

        telemetry.addData("Image Width", width);
        telemetry.addData("Image Height", height);

        telemetry.addData("Blue on Right", blueOnRight);

    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}