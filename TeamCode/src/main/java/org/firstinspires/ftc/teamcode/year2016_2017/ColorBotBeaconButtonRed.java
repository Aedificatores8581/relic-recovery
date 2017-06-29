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

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */

@Autonomous(name = "Beacon Button Red", group = "Color")
@Disabled
public class ColorBotBeaconButtonRed extends OpModeCamera {


// Touch sensor is "ts1".
    DcMotorController mc1;
    DcMotor left, right;
    int state = 0;
    int index = 0;
    UltrasonicSensor uss2;
    final int[][] positions = {{-1400, -700, -2000, 600, -2475, -700}, {-1400, 700, -2000, -600, -2475, 700}}; // Don't set these to zero. Ever.
    //final int[][] positions = {{400, 300, 600, 200}, {400, 300, -600, 200}}; // Don't set these to zero. Ever.
    int leftTargetPos, rightTargetPos, leftDirection, rightDirection;
    int leftPosition, rightPosition;
    int leftPosAftFirst, rightPosAftFirst;
    Queue<Double> rightUssValues;

    TouchSensor ts1, ts2;

    final double powah = 0.2;
    double correction = 1.0;
    boolean corrected;

    int doneState;
    Timer timer = new Timer();
    double observedSpeed;

    boolean step5timerSet = false;

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
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");
        ts1 = hardwareMap.touchSensor.get("ts1");
        ts2 = hardwareMap.touchSensor.get("ts2");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        state = 0;
        index = 0;

        rightUssValues = new LinkedList<>();
        for (int i = 0; i < 16; i++)
            rightUssValues.add(0.0);

        corrected = false;
        doneState = 0;
        launchedSpeed = false;
        step5timerSet = false;
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

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {
        if (!launchedSpeed) {
            timer.schedule(new SpeedTimerTask(left.getCurrentPosition(), 250, posProv, spdRecv, timer), 0);
            launchedSpeed = true;
        }

        switch (state) {
            case 0:
                leftPosition = positions[0][index];
                rightPosition = positions[1][index];
                if (index == 4) {
                    leftPosition -= leftPosAftFirst;
                    rightPosition -= rightPosAftFirst;
                }
                leftTargetPos = leftPosition + left.getCurrentPosition();
                rightTargetPos = rightPosition + right.getCurrentPosition();
                leftDirection = leftPosition / Math.abs(leftPosition);
                rightDirection = rightPosition / Math.abs(rightPosition);

                left.setPower(leftDirection * powah);
                right.setPower(rightDirection * powah * correction * Constants.RIGHT_POWER_FACTOR_2016);

                state = 1;

                break;

            case 1:

                if (leftDirection * (left.getCurrentPosition() - leftTargetPos) >= 0)
                    if (rightDirection * (right.getCurrentPosition() - rightTargetPos) >= 0) {
                        left.setPower(0.0f);
                        right.setPower(0.0f);

                        if (index == 0) {
                            leftPosAftFirst = left.getCurrentPosition();
                            rightPosAftFirst = right.getCurrentPosition();
                        }

                        state = 0;
                        index++;

                        if (!corrected) {
                            correction *= left.getCurrentPosition();
                            correction /= right.getCurrentPosition();
                            corrected = true;
                        }
                    }

                if (index >= positions[0].length) {
                    left.setPower(-powah * 0.5);
                    right.setPower(-powah * 0.5);
                    state = 2;
                }

                break;

            case 2:
                /*if (uss2.getUltrasonicLevel() <= 12) {
                    left.setPower(0);
                    right.setPower(0);
                    state = 3;
                }
                else if (getAverage(rightUssValues) <= 22) {
                    left.setPower(-powah * 0.25);
                    right.setPower(-powah * 0.25);
                }

                if (Math.abs(observedSpeed) < 25 && Math.abs(left.getPower()) > 0) {
                    left.setPower(0);
                    right.setPower(0);
                    state = 6;
                }*/

                if (ts1.isPressed() || ts2.isPressed()) {
                    left.setPower(powah * 0.5);
                    right.setPower(powah * 0.5);
                    state = 4;
                }

                break;
            case 3:
                if (getAverage(rightUssValues) > 12) {
                    left.setPower(-powah * 0.5);
                    right.setPower(-powah * 0.5);
                    state = 2;
                }
                else {
                    left.setPower(powah * 0.5);
                    right.setPower(powah * 0.5);
                    state = 4;
                }
                break;
            case 4:
                if (getAverage(rightUssValues) >= 25) {
                    left.setPower(0);
                    right.setPower(0);
                    state = 5;
                }
                break;
            case 5:
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                    /*left.setPower(-powah * 0.5);
                    right.setPower(-powah * 0.5);*/
                        state = 7;
                    }
                }, 5000);
                state = 6;
                break;
            case 6:
                break;
            case 7:
                if (getColor() == 2) {
                    left.setPower(-powah * 0.5);
                    right.setPower(-powah * 0.5);
                    state = 2;
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

        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

        telemetry.addData("Left Motor Pos.", left.getCurrentPosition());
        telemetry.addData("Right Motor Pos.", right.getCurrentPosition());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());

        telemetry.addData("USS Value", getAverage(rightUssValues));

        telemetry.addData("State", state);
        telemetry.addData("Index", index);

        telemetry.addData("Color", colorName(getColor()));
        telemetry.addData("Touch 1", ts1.isPressed());
        telemetry.addData("Touch 2", ts2.isPressed());
        telemetry.addData("Speed", observedSpeed);
    }

    @Override
    public void stop() {
        super.stop(); // stops camera functions
    }
}