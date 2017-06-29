package org.firstinspires.ftc.teamcode.year2016_2017;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import for_camera_opmodes.OpModeCamera;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "Autonomous Blue 2", group = "Autonomous")
@Disabled
public class AutoBotBlueTwo extends OpModeCamera {


// Touch sensor is "ts1".
    DcMotorController mc1;
    DcMotor left, right;
    int state = 0;
    int index = 0;
    UltrasonicSensor uss1, uss2;
    final int[][] positions = {{-410, -300, -3100, -350}, {-410, 300, -3100, 350}}; // Don't set these to zero. Ever.
    //final int[][] positions = {{400, 300, 600, 200}, {400, 300, -600, 200}}; // Don't set these to zero. Ever.
    int leftTargetPos, rightTargetPos, leftDirection, rightDirection;
    int leftPosition, rightPosition;
    int leftPosAftFirst, rightPosAftFirst;
    Queue<Double> leftUssValues, rightUssValues;

    TouchSensor ts1, ts2;

    final double powah = 0.2;
    double correction = 1.0;
    boolean corrected;


    Timer timer = new Timer();
    double observedSpeed;

    boolean beganMotors = false;
    boolean repeat = true;

    long timeBeforeFive;

    long beginningTime;
    boolean firstLoop;

    int avgCount, ussDiff;

    SpeedTimerTask.PositionProvider posProv = new SpeedTimerTask.PositionProvider() {
        @Override
        public double getPosition() {
            return left.getCurrentPosition();
        }
    };

    SpeedTimerTask.SpeedReceiver spdRecv = new SpeedTimerTask.SpeedReceiver() {
        @Override
        public void setSpeed(double speed) {
            observedSpeed = speed;
        }
    };

    boolean launchedSpeed = false;

    public double getCalcSpeed(int currPos) {
        final double MAX_SPEED = 0.4;
        final double MIN_SPEED = 0.1;

        final int MAX_POS = 1000;
        final int MIN_POS = 200;

        currPos = Math.max(200, Math.min(Math.abs(currPos), 1000));
        return currPos * (MAX_SPEED - MIN_SPEED) / (double)(MAX_POS - MIN_POS);
    }

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

        resetEncoders();

        left.setDirection(Constants.L_MTR_DIR_2016);
        right.setDirection(Constants.R_MTR_DIR_2016);

        state = 3;

        rightUssValues = new LinkedList<>();
        leftUssValues = new LinkedList<>();
        for (int i = 0; i < 16; i++) {
            rightUssValues.add(0.0);
            leftUssValues.add(0.0);
        }

        corrected = false;
        launchedSpeed = false;
        beganMotors = false;
        repeat = true;

        avgCount = 0;
        ussDiff = 0;
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
        int color = -1;
        if (imageReady()) {
            int redValue = 0;
            int blueValue = 0;
            int greenValue = 0;
            Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, 2);
            for (int x = 0; x < rgbImage.getWidth(); x++) {
                for (int y = 0; y < rgbImage.getHeight(); y++) {
                    int pixel = rgbImage.getPixel(x, y);
                    redValue += red(pixel);
                    blueValue += blue(pixel);
                    greenValue += green(pixel);
                }
            }
            color = highestColor(redValue, greenValue, blueValue);
        }
        return color;

    }

    public double getAverage(Queue<Double> dubs) {
        double sum = 0;
        int num = 0;
        for (double d : dubs) {
            sum += d;
            if (d != 0.0)
                num++;
        }
        return sum / (double)(num);
    }

    public void shiftQueue(Queue<Double> dubs, double dub) {
        if (dub != 0.0) {
            dubs.poll();
            dubs.add(dub);
        }
    }

    public void resetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {
        int color = getColor();

        if (firstLoop) {
            timer.schedule(new SpeedTimerTask(left.getCurrentPosition(), 250, posProv, spdRecv, timer), 0);
            beginningTime = System.currentTimeMillis();
            firstLoop = false;
        }

        /*if (!beganMotors) {
            left.setPower(powah * 0.5);
            right.setPower(-powah * 0.5);
            beganMotors = true;
        }*/

        /*left.setPower(powah * 0.5);
        right.setPower(-powah * 0.5);*/

        switch (state) {
            case 0: // Starts going towards the beacon.
                left.setPower(-powah * 0.5);
                right.setPower(-powah * 0.5);
                state = 1;
                break;
            case 1: // Still going towards the beacon.
                if (left.getCurrentPosition() <= -200 && right.getCurrentPosition() <= -200) {
                    resetEncoders();
                    left.setPower(-powah);
                    right.setPower(powah);
                    state = 2;
                }
                break;
            case 2: // Still going.
                if (left.getCurrentPosition() <= -300 && right.getCurrentPosition() >= 300) {
                    resetEncoders();
                    left.setPower(-powah);
                    right.setPower(-powah);
                    state = 3;
                }
                break;
            case 3: // Going on the long drive towards the beacon. This has the robot ramp down its power.
                if (left.getCurrentPosition() <= -3550 && right.getCurrentPosition() <= -3550) {
                    //left.setPower(-powah * 2.0);
                    //right.setPower(powah * 2.0);
                    resetEncoders();
                    left.setPower(-powah);
                    right.setPower(powah);
                    state = 4;
                    //left.setPower(0);
                    //right.setPower(0);
                    //state = 13;
                }
                else {
                    //left.setPower(-getCalcSpeed(left.getCurrentPosition() + 3100));
                    //right.setPower(-getCalcSpeed(right.getCurrentPosition() + 3100));
                    // 0.4 if >= 1000
                    // 0.2 if < 1000
                    // 0.1 if < 200
                    double power;
                    if (3100 + left.getCurrentPosition() < 50)
                        power = 0.15;
                    else if (3100 + left.getCurrentPosition() < 600)
                        power = 0.2;
                    else
                        power = 0.4;
                    left.setPower(-power * 0.98);
                    right.setPower(-power);
                    //left.setPower(-0.2);
                    //right.setPower(-0.2);
                }
                break;
            case 4: // Turns towards the beacon.
                if (left.getCurrentPosition() <= -360 && right.getCurrentPosition() >= 360) {
                    resetEncoders();
                    left.setPower(0);
                    right.setPower(0);
                    state = 5;
                }
                break;
            case 5: // Squares up with the wall.
                avgCount++;
                ussDiff += uss2.getUltrasonicLevel() - uss1.getUltrasonicLevel();
                if (avgCount == 10) {
                    ussDiff /= avgCount;
                    if (Math.abs(ussDiff) <= 2.0) {
                        //left.setPower(0);
                        //right.setPower(0);
                        //state = 13;
                        left.setPower(-powah);
                        right.setPower(-powah);
                        avgCount = 0;
                        ussDiff = 0;
                        state = 6;
                    }
                    else {
                        if (ussDiff < -2.0) {
                            left.setPower(powah * 0.5);
                            right.setPower(-powah * 0.5);
                        }
                        else {
                            left.setPower(-powah * 0.5);
                            right.setPower(powah * 0.5);
                        }

                        state = 9;
                    }
                }

                break;
            case 6:
                if (ts1.isPressed() || ts2.isPressed()) {
                    resetEncoders();

                    left.setPower(powah * 0.5);
                    right.setPower(powah * 0.5);
                    state = 7;
                }
                break;
            case 7:
                if ((left.getCurrentPosition() >= 400) && (right.getCurrentPosition() >= 400)) {
                    left.setPower(0.0);
                    right.setPower(0.0);

                    timeBeforeFive = System.currentTimeMillis();
                    state = 8;
                }
                break;
            case 8:
                if (color != 2) {

                    if (System.currentTimeMillis() - timeBeforeFive >= 5000) {
                        left.setPower(-powah);
                        right.setPower(-powah);
                        state = 6;
                    }
                }
                else if (repeat) {
                    resetEncoders();

                    left.setPower(powah);
                    right.setPower(-powah);

                    state = 10;
                }
                else {
                    left.setPower(0);
                    right.setPower(0);
                    state = 13;
                }
                break;
            case 9: // Makes sure that the ultrasonic values are correct. Used by state 5.
                if (uss2.getUltrasonicLevel() == uss1.getUltrasonicLevel()) {
                    left.setPower(0);
                    right.setPower(0);
                    ussDiff = 0;
                    avgCount = 0;
                    state = 5;
                }
                break;
            case 10:
                if (left.getCurrentPosition() >= 760 && right.getCurrentPosition() <= -760) {
                    resetEncoders();

                    left.setPower(-powah * 1.5);
                    right.setPower(-powah * 1.5);
                    state = 11;
                }
                break;
            case 11:
                if ((left.getCurrentPosition() <= -2600) && (right.getCurrentPosition() <= -2600)) {
                    resetEncoders();

                    left.setPower(-powah);
                    right.setPower(powah);
                    state = 12;
                }
                break;
            case 12:
                if (left.getCurrentPosition() <= -750 && right.getCurrentPosition() >= 750) {
                    resetEncoders();

                    left.setPower(0);
                    right.setPower(0);
                    repeat = false;
                    state = 5;
                }
                break;

            /*case 3:
                if (getAverage(rightUssValues) >= 20) {
                    left.setPower(0.0);
                    right.setPower(0.0);
                    state = 4;
                }
                else {
                    left.setPower(powah);
                    right.setPower(powah);
                }

                break;
            case 4:
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        state = 5;
                    }
                }, 6000);

                break;
            case 5:
                if (getColor() == 0) {
                    if (getAverage(rightUssValues) <= 6) {
                        left.setPower(0);
                        right.setPower(0);
                        timer.schedule(new TimerTask() {
                            @Override
                            public void run() {
                                doneState = 6;
                            }
                        }, 50);
                    }
                    else {
                        left.setPower(-powah / 4);
                        right.setPower(-powah / 4);
                    }
                }
                else
                    doneState = 7;

                break;
            case 6:
                if (getAverage(rightUssValues) >= 20) {
                    left.setPower(0.0);
                    right.setPower(0.0);
                    state = 7;
                }
                else {
                    left.setPower(powah);
                    right.setPower(powah);
                }

                break;*/

        }

        shiftQueue(leftUssValues, uss1.getUltrasonicLevel());
        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

        telemetry.addData("Left Motor Pos.", left.getCurrentPosition());
        telemetry.addData("Right Motor Pos.", right.getCurrentPosition());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());

        telemetry.addData("USS Right Value", getAverage(rightUssValues));
        telemetry.addData("USS Left Value", getAverage(leftUssValues));

        telemetry.addData("State", state);
        telemetry.addData("Index", index);

        telemetry.addData("Color", colorName(color));
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);

        telemetry.addData("Elapsed Time", (System.currentTimeMillis() - beginningTime) / 1000);

        telemetry.addData("Loop Count", avgCount);
        telemetry.addData("USS Diff", ussDiff);
    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}