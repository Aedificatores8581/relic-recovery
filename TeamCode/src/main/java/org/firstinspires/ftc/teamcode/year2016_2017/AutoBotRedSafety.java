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

import java.util.Arrays;
import java.util.Timer;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "Autonomous Red [S]", group = "Autonomous")
@Disabled
public class AutoBotRedSafety extends OpModeCamera {


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
    /*Queue<Double> leftUssValues, rightUssValues;*/
    Servo s1, s2;

    DcMotor bll, blr;
    TouchSensor ts1, ts2;

    final double powah = -0.165;
    double correction = 1.0;
    boolean corrected;

    Timer timer = new Timer();
    double observedSpeed;

    boolean beganMotors = false;
    boolean repeat = true;

    long timeBeforeFive;

    long beginningTime;
    boolean firstLoop;

    boolean alreadyFiredLauncher, alreadyGotColor;
    int color;

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

    public static final double LEFT_SPEED_FACTOR = 1.26 * (67.0 / 51.0);
    public static final double TICKS_FROM_INCHES = 3700.0 / 70.0;
    public static final int RIGHT_TARGET_DISTANCE = (int)(TICKS_FROM_INCHES * 56.0 * Math.PI / 2.0);
    public static final int LEFT_TARGET_DISTANCE = (int)(RIGHT_TARGET_DISTANCE / LEFT_SPEED_FACTOR);

    int avgCount, ussDiff;

    boolean readyToTurn = false, launchSecondBall = false, alreadyLaunchedSecond;

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
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");

        resetEncoders();

        left.setDirection(Constants.L_MTR_DIR_2016);
        right.setDirection(Constants.R_MTR_DIR_2016);

        state = 3;
        /*
        rightUssValues = new LinkedList<>();
        leftUssValues = new LinkedList<>();
        for (int i = 0; i < 16; i++) {
            rightUssValues.add(0.0);
            leftUssValues.add(0.0);
        }
        */
        corrected = false;
        launchedSpeed = false;
        beganMotors = false;
        repeat = true;

        firstLoop = true;
        alreadyFiredLauncher = false;

        color = -1;
        readyToTurn = false;

        alreadyLaunchedSecond = false;

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
        int redValue = 0;
        int blueValue = 0;
        int greenValue = 0;
        Bitmap rgbImage = convertYuvImageToRgb(yuvImage, width, height, 2);
        for (int x = 0; x < rgbImage.getWidth() / 2; x++) {
            for (int y = 0; y < rgbImage.getHeight() / 2; y++) {
                int pixel = rgbImage.getPixel(x, y);
                redValue += red(pixel);
                blueValue += blue(pixel);
                greenValue += green(pixel);
            }
        }
        return highestColor(redValue, greenValue, blueValue);
    }

    /*public double getAverage(Queue<Double> dubs) {
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
    }*/

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

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {

        if (firstLoop) {
            timer.schedule(new SpeedTimerTask(getLeftPos(), 250, posProv, spdRecv, timer), 0);
            beginningTime = System.currentTimeMillis();
            s1.setPosition(Constants.SERVO_MIN_2016);
            s2.setPosition(0.5);
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
            case 3: // Going on the long drive towards the beacon. This has the robot ramp down its power.
                if (getLeftPos() >= RIGHT_TARGET_DISTANCE * Constants.ENCODER_MULT && getRightPos() >= LEFT_TARGET_DISTANCE * Constants.ENCODER_MULT) {
                    //left.setPower(-powah * 2.0);
                    //right.setPower(powah * 2.0);
                    resetEncoders();
                    left.setPower(powah * 0.675);
                    right.setPower(powah * 0.675);
                    state = 6;
                    //left.setPower(0);
                    //right.setPower(0);
                    //state = 13;
                }
                else {
                    //left.setPower(-getCalcSpeed(getLeftPos() + 3100));
                    //right.setPower(-getCalcSpeed(getRightPos() + 3100));
                    double power = 0.3;
                    left.setPower(-power);
                    right.setPower(-power / LEFT_SPEED_FACTOR);
                    //left.setPower(-0.2);
                    //right.setPower(-0.2);
                }
                break;
            case 6: // Drives forwards until it hits the button.
                if (ts1.isPressed() || ts2.isPressed()) {
                    resetEncoders();

                    left.setPower(-powah);
                    right.setPower(-powah);

                    if (!alreadyFiredLauncher) {
                        bll.setPower(-0.4);
                        blr.setPower(-0.4);
                    }

                    timeBeforeFive = System.currentTimeMillis();
                    state = 7;
                }
                break;
            case 7: // Backing up and launching ball.
                if ((getLeftPos() <= -640 * Constants.ENCODER_MULT) && (getRightPos() <= -640 * Constants.ENCODER_MULT)) {
                    left.setPower(0.0);
                    right.setPower(0.0);

                    if (!alreadyFiredLauncher)
                        s1.setPosition(Constants.SERVO_MAX_2016);

                    if (!alreadyFiredLauncher && !alreadyLaunchedSecond) {
                        launchSecondBall = true;
                        alreadyLaunchedSecond = true;
                    }

                    alreadyGotColor = false;

                    state = 8;
                }
                break;
            case 8: // Checks color.
                if (System.currentTimeMillis() - timeBeforeFive >= 2100) {
                    while (!alreadyGotColor) {
                        color = getColor();
                        if (Arrays.asList(0, 2).contains(color))
                            alreadyGotColor = true;
                    }
                    if (readyToTurn) {
                        if (color == 0) {
                            resetEncoders();

                            left.setPower(-powah * 1.1);
                            right.setPower(powah * 1.1);

                            if (repeat)
                                state = 10;
                            else
                                state = 13;
                        }
                        else if (System.currentTimeMillis() - timeBeforeFive >= 5750) {
                            left.setPower(powah);
                            right.setPower(powah);
                            state = 6;
                        }
                    }
                }

                break;
            case 10: // Turning away from the beacon.
                if (getLeftPos() <= -620 * Constants.ENCODER_MULT && getRightPos() >= 620 * Constants.ENCODER_MULT) {
                    resetEncoders();

                    left.setPower(powah * 1.5);
                    right.setPower(powah * 1.5);
                    state = 11;
                }
                break;
            case 11: // Driving towards the next beacon.
                if ((getLeftPos() >= 2400 * Constants.ENCODER_MULT) && (getRightPos() >= 2400 * Constants.ENCODER_MULT)) {
                    resetEncoders();

                    left.setPower(powah * 1.1);
                    right.setPower(-powah * 1.1);
                    state = 12;
                }
                break;
            case 12: // Turns towards second beacon.
                if (getLeftPos() >= 580 * Constants.ENCODER_MULT && getRightPos() <= -580 * Constants.ENCODER_MULT) {
                    resetEncoders();

                    repeat = false;
                    if ((System.currentTimeMillis() - beginningTime) <= 2300) {
                        left.setPower(powah);
                        right.setPower(powah);
                        state = 15;
                    }
                    else {
                        left.setPower(-powah * 1.1);
                        right.setPower(powah * 1.1);
                        state = 13;
                    }
                }
                break;
            case 13: // Turn to go hit the ball.
                if (getLeftPos() <= -240 && getRightPos() >= 240) {
                    resetEncoders();

                    left.setPower(-powah * 1.5);
                    right.setPower(-powah * 1.5);

                    state = 14;
                }
                break;
            case 14: // Go hit the ball.
                if (getLeftPos() <= -2400 && getRightPos() <= -2400) {
                    resetEncoders();

                    left.setPower(0);
                    right.setPower(0);

                    state = -1;
                }
                break;
            case 15: // Squares up with the wall.
                avgCount++;
                ussDiff += uss2.getUltrasonicLevel() - uss1.getUltrasonicLevel();
                if (avgCount == 25) {
                    ussDiff /= avgCount;
                    if (Math.abs(ussDiff) <= 2.0) {
                        //left.setPower(0);
                        //right.setPower(0);
                        //state = 13;
                        left.setPower(powah);
                        right.setPower(powah);
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

                        state = 16;
                    }
                }

                break;
            case 16: // Makes sure that the ultrasonic values are correct. Used by state 15.
                if (uss2.getUltrasonicLevel() == uss1.getUltrasonicLevel()) {
                    left.setPower(0);
                    right.setPower(0);
                    ussDiff = 0;
                    avgCount = 0;
                    state = 15;
                }
                break;
        }

        if (launchSecondBall) {
            if (System.currentTimeMillis() - timeBeforeFive >= 5600) {
                bll.setPower(0);
                blr.setPower(0);
                s1.setPosition(Constants.SERVO_MIN_2016);
                readyToTurn = true;
                launchSecondBall = false;
                alreadyFiredLauncher = true;
            }
            else if (System.currentTimeMillis() - timeBeforeFive >= 4600) {
                s1.setPosition(Constants.SERVO_MAX_2016);
            }
            else if (System.currentTimeMillis() - timeBeforeFive >= 3600) {
                s2.setPosition(0.35);
            }
            else if (System.currentTimeMillis() - timeBeforeFive >= 3100) {
                s1.setPosition(Constants.SERVO_MIN_2016);
            }
        }

        /*shiftQueue(leftUssValues, uss1.getUltrasonicLevel());
        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

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

        telemetry.addData("State", state);
        telemetry.addData("Index", index);

        */
        if (color != -1)
            telemetry.addData("Color", colorName(color));
        /*
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);

        telemetry.addData("Elapsed Time", (System.currentTimeMillis() - beginningTime) / 1000l);
        */
    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}