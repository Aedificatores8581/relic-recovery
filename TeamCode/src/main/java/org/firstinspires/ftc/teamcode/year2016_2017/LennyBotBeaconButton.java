package org.firstinspires.ftc.teamcode.year2016_2017;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 * @version 1.0
 */
//@Autonomous(name = "LennyBot: Beacon Button Pusher", group = "( \u0361\u00B0 \u035C\u0296 \u0361\u00B0 )")
public class LennyBotBeaconButton extends OpMode
{
    DcMotorController mc1;
    //DcMotorController con;
    //DcMotor motor1, motor2;
    //DcMotor motor3, motor4;
    DcMotor left, right;
    //TouchSensor touch1;
    //IrSeekerSensor irseek1;
    //OpticalDistanceSensor uss1;
    int state = 0;
    UltrasonicSensor uss2;
    final int[][] positions = {{-1400, 700, -2000, -600, -2475, 700}, {-1400, -700, -2000, 600, -2475, -700}}; // Don't set these to zero. Ever.
    //final int[][] positions = {{400, 300, 600, 200}, {400, 300, -600, 200}}; // Don't set these to zero. Ever.
    int leftTargetPos, rightTargetPos, leftDirection, rightDirection;
    int leftPosition, rightPosition;
    int leftPosAftFirst, rightPosAftFirst;
    Queue<Double> rightUssValues;

    final double powah = 0.2;
    double correction = 1.0;
    boolean corrected;

    int doneState;
    Timer timer = new Timer();

    //--------------------------------------------------------------------------
    //
    // PootisBotManual
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public LennyBotBeaconButton()
    {


    } // PootisBotManual::PootisBotManual

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        mc1 = hardwareMap.dcMotorController.get("mc1");
        uss2 = hardwareMap.ultrasonicSensor.get("uss2");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        state = 0;

        rightUssValues = new LinkedList<>();
        for (int i = 0; i < 64; i++)
            rightUssValues.add(0.0);

        corrected = false;
        doneState = 0;
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

    //--------------------------------------------------------------------------
    //
    // loop
    //
    //-------
    // Initializes the class.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override public void loop ()
    {

/*

        position = positions[state];
        targetPos = position + left.getCurrentPosition();
        direction = position / Math.abs(position);
        if (left.getPower()==0) {

            left.setPower(direction);
            right.setPower(direction);
        }
        if (direction * (left.getCurrentPosition() - targetPos) >= 0) {
            left.setPower(0.0f);
            right.setPower(0.0f);
            state++;
        }

        */
        if (state % 2 == 0) {
            leftPosition = positions[0][state / 2];
            rightPosition = positions[1][state / 2];
            if (state == 8) {
                leftPosition -= leftPosAftFirst;
                rightPosition -= rightPosAftFirst;
            }
            leftTargetPos = leftPosition + left.getCurrentPosition();
            rightTargetPos = rightPosition + right.getCurrentPosition();
            leftDirection = leftPosition / Math.abs(leftPosition);
            rightDirection = rightPosition / Math.abs(rightPosition);

            left.setPower(leftDirection * powah);
            right.setPower(rightDirection * powah * correction);

            state++;
        }

        if (state % 2 == 1) {
            if (leftDirection * (left.getCurrentPosition() - leftTargetPos) >= 0)
                if (rightDirection * (right.getCurrentPosition() - rightTargetPos) >= 0) {
                    left.setPower(0.0f);
                    right.setPower(0.0f);

                    if (state == 1) {
                        leftPosAftFirst = left.getCurrentPosition();
                        rightPosAftFirst = right.getCurrentPosition();
                    }

                    state++;

                    if (!corrected) {
                        correction *= left.getCurrentPosition();
                        correction /= right.getCurrentPosition();
                        corrected = true;
                    }
                }
        }

        if (state % 2 == -1) {
            if (doneState == 0)
                if (getAverage(rightUssValues) <= 8.0) {
                    timer.schedule(new TimerTask() {
                        @Override
                        public void run() {
                            left.setPower(0.0);
                            right.setPower(0.0);
                            timer.schedule(new TimerTask() {
                                @Override
                                public void run() {
                                    doneState = 1;
                                }
                            }, 500);
                        }
                    }, 500);
                }
                else {
                    left.setPower(-powah);
                    right.setPower(-powah);
                }
            else if (doneState == 1) {
                if (getAverage(rightUssValues) >= 20) {
                    left.setPower(0.0);
                    right.setPower(0.0);
                    doneState = 2;
                }
                else {
                    left.setPower(powah);
                    right.setPower(powah);
                }
            }
        }

        if ((state / 2) >= positions[0].length) {
            state = -1;
        }

        shiftQueue(rightUssValues, uss2.getUltrasonicLevel());

        telemetry.addData("Left Motor Pos.", left.getCurrentPosition());
        telemetry.addData("Right Motor Pos.", right.getCurrentPosition());

        telemetry.addData("Left Motor Pow.", left.getPower());
        telemetry.addData("Right Motor Pow.", right.getPower());

    } // PootisBotManual::loop

    @Override
    public void stop() {
        timer.purge();
    }

} // PootisBotManual
